from __future__ import annotations

import struct
import sys
import re
import time
from collections import deque
from typing import Callable

import numpy as np
import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSplitter,
    QVBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QWidget,
)


# -----------------------------------------------------------------------------
# Protocol constants and helpers
# -----------------------------------------------------------------------------

UART_PORT = "COM11"
UART_BAUDRATE = 115200

HOST_SYNC = b"\xA5\x5A"
DEVICE_SYNC = b"\x5A\xA5"
CMD_CONFIG = 0x01
CMD_RUN = 0x02
CMD_STOP = 0x03
PKT_DATA = 0x10
PKT_STATUS = 0x11
STATUS_COMPLETE = 0x01

TRIGGER_CODES = {
    "Không trigger": 0,
    "Cạnh lên (0→1)": 1,
    "Cạnh xuống (1→0)": 2,
    "Mức HIGH": 3,
    "Mức LOW": 4,
}

SAMPLE_RATES = [
    ("100 kS/s", 100_000),
    ("500 kS/s", 500_000),
    ("1 MS/s", 1_000_000),
    ("5 MS/s", 5_000_000),
    ("10 MS/s", 10_000_000),
    ("25 MS/s", 25_000_000),
    ("50 MS/s", 50_000_000),
    ("100 MS/s", 100_000_000),
]

FIXED_BUFFER_SIZE = 256
MAX_DISPLAY_SAMPLES = 4096


class FramedParser:
    """Incremental parser for 5A A5 TYPE LEN_LO LEN_HI PAYLOAD frames."""

    MAX_PAYLOAD = 65535

    def __init__(self, on_data: Callable[[bytes], None], on_status: Callable[[int], None]):
        self.buffer = bytearray()
        self.on_data = on_data
        self.on_status = on_status

    def reset(self) -> None:
        self.buffer.clear()

    def feed(self, chunk: bytes) -> None:
        if not chunk:
            return
        self.buffer.extend(chunk)

        while True:
            sync_at = self.buffer.find(DEVICE_SYNC)
            if sync_at < 0:
                keep = 1 if self.buffer[-1:] == DEVICE_SYNC[:1] else 0
                if keep:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                return

            if sync_at:
                del self.buffer[:sync_at]

            if len(self.buffer) < 5:
                return

            packet_type = self.buffer[2]
            payload_len = self.buffer[3] | (self.buffer[4] << 8)
            if payload_len > self.MAX_PAYLOAD:
                del self.buffer[0]
                continue

            frame_len = 5 + payload_len
            if len(self.buffer) < frame_len:
                return

            payload = bytes(self.buffer[5:frame_len])
            del self.buffer[:frame_len]

            if packet_type == PKT_DATA:
                self.on_data(payload)
            elif packet_type == PKT_STATUS and payload:
                self.on_status(payload[0])


def build_config_frame(sample_rate: int, trigger_channel: int, trigger_mode: int,
                       pre_samples: int, post_samples: int) -> bytes:
    """Build the 13-byte PC -> FPGA configuration packet."""
    fields = struct.pack(
        "<IBBHH",
        int(sample_rate),
        int(trigger_channel),
        int(trigger_mode),
        int(pre_samples),
        int(post_samples),
    )
    return HOST_SYNC + bytes([CMD_CONFIG]) + fields


def build_command(command: int) -> bytes:
    return HOST_SYNC + bytes([command & 0xFF])


class RawTriggerCapture:
    """Software trigger/capture helper used only in raw-stream mode."""

    def __init__(self, trigger_channel: int, trigger_mode: int,
                 pre_samples: int, post_samples: int):
        self.channel = trigger_channel
        self.mode = trigger_mode
        self.pre_count = pre_samples
        self.post_count = post_samples
        self.pre_buffer: deque[int] = deque(maxlen=max(1, pre_samples))
        self.previous: int | None = None
        self.triggered = False
        self.post_remaining = post_samples
        self.samples = bytearray()
        self.complete = False
        self.target_count = max(1, pre_samples + post_samples + 1)

    def _match(self, current: int) -> bool:
        bit = (current >> self.channel) & 1
        if self.mode == 0:
            return True
        if self.mode == 1:
            return self.previous is not None and ((self.previous >> self.channel) & 1) == 0 and bit == 1
        if self.mode == 2:
            return self.previous is not None and ((self.previous >> self.channel) & 1) == 1 and bit == 0
        if self.mode == 3:
            return bit == 1
        if self.mode == 4:
            return bit == 0
        return False

    def feed(self, chunk: bytes) -> bytes:
        newly_captured = bytearray()
        for raw in chunk:
            if self.complete:
                break
            value = int(raw)

            if not self.triggered:
                if self._match(value):
                    self.triggered = True
                    if self.mode == 0:
                        self.samples.append(value)
                        newly_captured.append(value)
                        remaining = self.target_count - len(self.samples)
                        self.post_remaining = max(0, remaining)
                    else:
                        pre = list(self.pre_buffer)[-self.pre_count:] if self.pre_count else []
                        self.samples.extend(pre)
                        self.samples.append(value)
                        newly_captured.extend(pre)
                        newly_captured.append(value)
                        self.post_remaining = self.post_count
                    if self.post_remaining == 0:
                        self.complete = True
                else:
                    if self.pre_count > 0:
                        self.pre_buffer.append(value)
                self.previous = value
            else:
                if self.post_remaining > 0:
                    self.samples.append(value)
                    newly_captured.append(value)
                    self.post_remaining -= 1
                if self.post_remaining == 0:
                    self.complete = True
                self.previous = value

        return bytes(newly_captured)


# -----------------------------------------------------------------------------
# Offline protocol decoders (operate on captured 8-bit digital samples)
# -----------------------------------------------------------------------------

MAX_DECODE_ROWS = 500


def _time_label(sample_index: int, sample_rate: float) -> str:
    return f"{sample_index / sample_rate * 1e6:.3f} µs"


def _bits_for_channel(samples: np.ndarray, channel: int) -> np.ndarray:
    return ((samples >> int(channel)) & 1).astype(np.uint8)


def _make_event(start: int, end: int, sample_rate: float, protocol: str,
                data: str, details: str) -> dict:
    return {
        "start": _time_label(start, sample_rate),
        "end": _time_label(end, sample_rate),
        "protocol": protocol,
        "data": data,
        "details": details,
    }


def decode_uart(samples: np.ndarray, sample_rate: float, channel: int,
                baud_rate: int) -> tuple[list[dict], str]:
    """Decode asynchronous UART 8N1, LSB-first, from sampled RX logic."""
    if samples.size < 4:
        return [], "Cần thêm mẫu để giải mã UART."
    if baud_rate <= 0:
        return [], "Baud rate không hợp lệ."

    samples_per_bit = sample_rate / float(baud_rate)
    if samples_per_bit < 2.0:
        return [], (f"Không đủ độ phân giải: {samples_per_bit:.2f} mẫu/bit. "
                    "Nên lấy mẫu ít nhất 4–8 lần baud rate.")

    line = _bits_for_channel(samples, channel)
    events: list[dict] = []
    n = len(line)
    i = 1
    frame_span = int(np.ceil(10.0 * samples_per_bit))

    while i < n - int(9.5 * samples_per_bit):
        # UART idle is HIGH; a falling edge marks the beginning of a start bit.
        if line[i - 1] == 1 and line[i] == 0:
            start = i
            start_center = int(round(start + 0.5 * samples_per_bit))
            if start_center >= n or line[start_center] != 0:
                i += 1
                continue

            value = 0
            for bit_index in range(8):
                center = int(round(start + (1.5 + bit_index) * samples_per_bit))
                if center >= n:
                    break
                value |= int(line[center]) << bit_index
            else:
                stop_center = int(round(start + 9.5 * samples_per_bit))
                frame_end = min(n - 1, int(round(start + 10.0 * samples_per_bit)))
                stop_ok = stop_center < n and line[stop_center] == 1
                char = chr(value) if 32 <= value <= 126 else "·"
                detail = f"8N1 · ASCII '{char}' · Stop {'OK' if stop_ok else 'LOW'}"
                events.append(_make_event(start, frame_end, sample_rate, "UART",
                                          f"0x{value:02X}", detail))
                if len(events) >= MAX_DECODE_ROWS:
                    break
                i = max(i + 1, start + frame_span)
                continue
        i += 1

    message = f"UART D{channel}, {baud_rate:,} baud, 8N1 · {len(events)} byte"
    if not events:
        message += " · Không tìm thấy frame hợp lệ; kiểm tra RX/baud/sample rate."
    return events, message


def decode_spi(samples: np.ndarray, sample_rate: float, sclk_ch: int,
               mosi_ch: int, miso_ch: int, cs_ch: int,
               cpol: int, cpha: int, msb_first: bool) -> tuple[list[dict], str]:
    """Decode SPI bytes on selected clock/data lines."""
    if samples.size < 2:
        return [], "Cần thêm mẫu để giải mã SPI."

    clk = _bits_for_channel(samples, sclk_ch)
    mosi = _bits_for_channel(samples, mosi_ch)
    miso = _bits_for_channel(samples, miso_ch) if miso_ch >= 0 else None
    cs = _bits_for_channel(samples, cs_ch) if cs_ch >= 0 else None

    events: list[dict] = []
    tx_bits: list[int] = []
    rx_bits: list[int] = []
    byte_start = 0

    def to_byte(bits: list[int]) -> int:
        value = 0
        if msb_first:
            for bit in bits:
                value = (value << 1) | int(bit)
        else:
            for index, bit in enumerate(bits):
                value |= int(bit) << index
        return value

    for i in range(1, len(samples)):
        selected = cs is None or cs[i] == 0  # CS active LOW
        if not selected:
            tx_bits.clear()
            rx_bits.clear()
            continue

        previous_clock, current_clock = int(clk[i - 1]), int(clk[i])
        if previous_clock == current_clock:
            continue

        leading_edge = previous_clock == cpol and current_clock != cpol
        trailing_edge = previous_clock != cpol and current_clock == cpol
        sample_edge = leading_edge if cpha == 0 else trailing_edge
        if not sample_edge:
            continue

        if not tx_bits:
            byte_start = i
        tx_bits.append(int(mosi[i]))
        if miso is not None:
            rx_bits.append(int(miso[i]))

        if len(tx_bits) == 8:
            tx_value = to_byte(tx_bits)
            rx_text = f"MISO=0x{to_byte(rx_bits):02X}" if miso is not None else "MISO không chọn"
            events.append(_make_event(
                byte_start, i, sample_rate, "SPI", f"MOSI=0x{tx_value:02X}",
                f"{rx_text} · CPOL={cpol}, CPHA={cpha} · "
                f"{'MSB' if msb_first else 'LSB'} first"
            ))
            tx_bits.clear()
            rx_bits.clear()
            if len(events) >= MAX_DECODE_ROWS:
                break

    message = (f"SPI · SCLK=D{sclk_ch}, MOSI=D{mosi_ch}, "
               f"{'MISO=D' + str(miso_ch) if miso_ch >= 0 else 'không MISO'} · "
               f"CPOL={cpol}/CPHA={cpha} · {len(events)} byte")
    if not events:
        message += " · Không tìm thấy đủ 8 cạnh clock cho một byte."
    return events, message


def decode_i2c(samples: np.ndarray, sample_rate: float,
               scl_ch: int, sda_ch: int) -> tuple[list[dict], str]:
    """Decode I2C START/STOP, address/data bytes and ACK/NACK."""
    if samples.size < 3:
        return [], "Cần thêm mẫu để giải mã I2C."

    scl = _bits_for_channel(samples, scl_ch)
    sda = _bits_for_channel(samples, sda_ch)
    events: list[dict] = []
    active = False
    first_byte = True
    collected: list[tuple[int, int]] = []
    transaction_start = 0

    for i in range(1, len(samples)):
        # START / repeated START / STOP: SDA changes while SCL stays HIGH.
        if scl[i - 1] == 1 and scl[i] == 1 and sda[i - 1] != sda[i]:
            if sda[i - 1] == 1 and sda[i] == 0:  # START
                active = True
                first_byte = True
                collected.clear()
                transaction_start = i
            elif sda[i - 1] == 0 and sda[i] == 1 and active:  # STOP
                if collected and len(events) < MAX_DECODE_ROWS:
                    bits_text = "".join(str(bit) for _, bit in collected)
                    events.append(_make_event(
                        collected[0][0], i, sample_rate, "I2C", "Partial",
                        f"STOP · byte chưa đủ 9 clock: {bits_text}"
                    ))
                active = False
                collected.clear()
                continue

        # I2C samples SDA on SCL rising edges.
        if active and scl[i - 1] == 0 and scl[i] == 1:
            collected.append((i, int(sda[i])))
            if len(collected) == 9:
                data_bits = [bit for _, bit in collected[:8]]
                value = 0
                for bit in data_bits:
                    value = (value << 1) | bit
                ack = collected[8][1] == 0
                ack_text = "ACK" if ack else "NACK"
                start_index, end_index = collected[0][0], collected[-1][0]
                if first_byte:
                    address = (value >> 1) & 0x7F
                    rw = "R" if value & 1 else "W"
                    data_text = f"ADDR 0x{address:02X} ({rw})"
                    detail = f"7-bit address · {ack_text}"
                    first_byte = False
                else:
                    data_text = f"0x{value:02X}"
                    char = chr(value) if 32 <= value <= 126 else "·"
                    detail = f"DATA · ASCII '{char}' · {ack_text}"
                events.append(_make_event(start_index, end_index, sample_rate,
                                          "I2C", data_text, detail))
                collected.clear()
                if len(events) >= MAX_DECODE_ROWS:
                    break

    message = f"I2C · SCL=D{scl_ch}, SDA=D{sda_ch} · {len(events)} byte/event"
    if not events:
        message += " · Không phát hiện START/clock/ACK; kiểm tra mapping SCL/SDA."
    return events, message


def decode_pwm(samples: np.ndarray, sample_rate: float,
               channel: int) -> tuple[list[dict], str]:
    """Measure PWM period, high time, frequency and duty cycle."""
    if samples.size < 3:
        return [], "Cần thêm mẫu để giải mã PWM."

    line = _bits_for_channel(samples, channel)
    rising = np.flatnonzero((line[1:] == 1) & (line[:-1] == 0)) + 1
    falling = np.flatnonzero((line[1:] == 0) & (line[:-1] == 1)) + 1
    events: list[dict] = []

    for index in range(len(rising) - 1):
        start = int(rising[index])
        next_rise = int(rising[index + 1])
        fall_candidates = falling[(falling > start) & (falling < next_rise)]
        if not len(fall_candidates):
            continue
        fall = int(fall_candidates[0])
        period_samples = next_rise - start
        high_samples = fall - start
        frequency = sample_rate / period_samples
        duty = 100.0 * high_samples / period_samples
        events.append(_make_event(
            start, next_rise, sample_rate, "PWM",
            f"{frequency:.3f} Hz",
            f"T={period_samples / sample_rate * 1e6:.3f} µs · "
            f"HIGH={high_samples / sample_rate * 1e6:.3f} µs · Duty={duty:.2f}%"
        ))
        if len(events) >= MAX_DECODE_ROWS:
            break

    message = f"PWM · D{channel} · {len(events)} chu kỳ đo được"
    if not events:
        message += " · Chưa thấy đủ hai cạnh lên và một cạnh xuống."
    return events, message


def decode_raw_transitions(samples: np.ndarray, sample_rate: float) -> tuple[list[dict], str]:
    """Show unclassified logic transitions as hex samples and changed channels."""
    if samples.size == 0:
        return [], "Chưa có dữ liệu."

    events: list[dict] = [
        _make_event(0, 0, sample_rate, "RAW", f"0x{int(samples[0]):02X}", "Trạng thái ban đầu")
    ]
    for i in range(1, len(samples)):
        old, new = int(samples[i - 1]), int(samples[i])
        changed = old ^ new
        if changed == 0:
            continue
        names = []
        for channel in range(8):
            mask = 1 << channel
            if changed & mask:
                names.append(f"D{channel}{'↑' if new & mask else '↓'}")
        events.append(_make_event(
            i, i, sample_rate, "RAW", f"0x{new:02X}",
            "Chuyển mức: " + ", ".join(names)
        ))
        if len(events) >= MAX_DECODE_ROWS:
            break

    return events, f"Tín hiệu tự do/ngẫu nhiên · {len(events) - 1} lần thay đổi trạng thái"


# -----------------------------------------------------------------------------
# Plot interaction
# -----------------------------------------------------------------------------

class HorizontalViewBox(pg.ViewBox):
    """ViewBox that lets the window know when the user manually zooms/pans."""

    def __init__(self, interaction_callback: Callable[[], None] | None = None):
        super().__init__()
        self.interaction_callback = interaction_callback

    def wheelEvent(self, ev, axis=None):
        if self.interaction_callback:
            self.interaction_callback()
        super().wheelEvent(ev, axis=axis)

    def mouseDragEvent(self, ev, axis=None):
        if self.interaction_callback and ev.button() == Qt.MouseButton.LeftButton:
            self.interaction_callback()
        super().mouseDragEvent(ev, axis=axis)


# -----------------------------------------------------------------------------
# Main window
# -----------------------------------------------------------------------------

class LogicAnalyzerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Logic Analyzer GUI")
        self.resize(1480, 900)
        self.setMinimumSize(1120, 700)

        self.ser: serial.Serial | None = None
        self.acquiring = False
        self.capture_data = bytearray()
        # converter.v sends each sample as: timestamp byte, then 8-bit DATA byte.
        # Keep an odd trailing byte between serial reads so pairs are never misaligned.
        self.rx_pair_buffer = bytearray()
        self.total_uart_bytes = 0
        self.total_pairs_received = 0
        self.total_received = 0
        self.batch_size = FIXED_BUFFER_SIZE
        self.batch_received = 0
        self.last_rx_time = 0.0
        self.target_sample_count = FIXED_BUFFER_SIZE
        self.auto_stop_scheduled = False
        self.has_new_data = False

        self.raw_capture: RawTriggerCapture | None = None
        self.protocol_mode = "framed"
        self.sample_rate = 1_000_000
        self.auto_follow = True
        self.decoder_row_cache: list[dict[str, str]] = []
        self.curves: list[pg.PlotDataItem] = []

        self.parser = FramedParser(self._on_framed_data, self._on_device_status)

        self._build_ui()
        self._apply_style()

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(8)
        self.poll_timer.timeout.connect(self._poll_serial)

        self.watchdog_timer = QTimer(self)
        self.watchdog_timer.setInterval(500)
        self.watchdog_timer.timeout.connect(self._check_port_alive)

        self._set_running_ui(False)
        self._set_status(f"Chưa kết nối ({UART_PORT})", "#e0a84b")

    # ----- UI construction ----------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        # Header
        header = QHBoxLayout()
        brand = QLabel("▥ LOGIC ANALYZER")
        brand.setObjectName("brand")
        header.addWidget(brand)
        header.addStretch(1)
        self.top_status = QLabel("● OFFLINE")
        self.top_status.setObjectName("statusPill")
        header.addWidget(self.top_status)
        root.addLayout(header)

        # One compact row: connection/actions + sampling + pre/post + trigger.
        action_frame = QFrame()
        action_frame.setObjectName("actionBar")
        action = QHBoxLayout(action_frame)
        action.setContentsMargins(8, 6, 8, 6)
        action.setSpacing(5)

        self.connect_button = QPushButton("Connect")
        self.connect_button.setObjectName("connectButton")
        self.connect_button.setToolTip(f"Kết nối thiết bị qua {UART_PORT}")
        self.connect_button.setFixedSize(82, 32)
        self.connect_button.clicked.connect(self.connect_uart)

        self.run_button = QPushButton("▶ RUN")
        self.run_button.setObjectName("runButton")
        self.run_button.setFixedSize(64, 32)
        self.run_button.clicked.connect(self.start_acquisition)

        self.stop_button = QPushButton("■ STOP")
        self.stop_button.setObjectName("stopButton")
        self.stop_button.setFixedSize(66, 32)
        self.stop_button.clicked.connect(lambda: self.stop_acquisition(send_command=True))
        self.stop_button.setEnabled(False)

        action.addWidget(self.connect_button)
        action.addWidget(self.run_button)
        action.addWidget(self.stop_button)
        action.addSpacing(4)

        fs_label = QLabel("Fs")
        fs_label.setObjectName("inlineFieldLabel")
        action.addWidget(fs_label)
        self.rate_combo = QComboBox()
        for label, rate in SAMPLE_RATES:
            self.rate_combo.addItem(label, rate)
        self.rate_combo.setCurrentIndex(2)
        self.rate_combo.setFixedWidth(94)
        action.addWidget(self.rate_combo)

        pre_label = QLabel("Pre/Post")
        pre_label.setObjectName("inlineFieldLabel")
        action.addWidget(pre_label)
        self.pre_post_slider = QSlider(Qt.Orientation.Horizontal)
        self.pre_post_slider.setObjectName("sampleSlider")
        self.pre_post_slider.setRange(0, 100)
        self.pre_post_slider.setSingleStep(5)
        self.pre_post_slider.setPageStep(10)
        self.pre_post_slider.setValue(0)
        self.pre_post_slider.setFixedWidth(105)
        self.pre_post_value_label = QLabel()
        self.pre_post_value_label.setObjectName("sliderValue")
        self.pre_post_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pre_post_value_label.setFixedWidth(78)

        def update_pre_post_label(*_args) -> None:
            pre = self._selected_pre_samples()
            post = self._selected_post_samples()
            self.pre_post_value_label.setText(f"{pre}/{post}")
            self.pre_post_value_label.setToolTip(
                f"Pre-trigger: {pre:,} mẫu · Post-trigger: {post:,} mẫu"
            )

        self.pre_post_slider.valueChanged.connect(update_pre_post_label)
        update_pre_post_label()
        action.addWidget(self.pre_post_slider)
        action.addWidget(self.pre_post_value_label)

        trigger_label = QLabel("Trig")
        trigger_label.setObjectName("inlineFieldLabel")
        action.addWidget(trigger_label)
        self.trigger_channel_combo = QComboBox()
        for i in range(8):
            self.trigger_channel_combo.addItem(f"D{i}", i)
        self.trigger_channel_combo.setFixedWidth(58)
        action.addWidget(self.trigger_channel_combo)

        self.trigger_mode_combo = QComboBox()
        for label, code in TRIGGER_CODES.items():
            self.trigger_mode_combo.addItem(label, code)
        self.trigger_mode_combo.setCurrentIndex(0)
        self.trigger_mode_combo.setFixedWidth(136)
        action.addWidget(self.trigger_mode_combo)

        self.capture_label = QLabel("Chưa thu thập dữ liệu")
        self.capture_label.setObjectName("muted")
        self.capture_label.setMinimumWidth(0)
        self.capture_label.setMaximumWidth(205)
        self.capture_label.setToolTip("Trạng thái thu thập dữ liệu UART")
        action.addWidget(self.capture_label, 1)
        root.addWidget(action_frame)

        # Waveform above, 8-channel protocol/data table below.
        content_splitter = QSplitter(Qt.Orientation.Vertical)
        content_splitter.setChildrenCollapsible(False)
        content_splitter.addWidget(self._build_waveform_panel())
        content_splitter.addWidget(self._build_decoder_panel())
        content_splitter.setStretchFactor(0, 3)
        content_splitter.setStretchFactor(1, 2)
        content_splitter.setSizes([480, 340])
        root.addWidget(content_splitter, 1)

        self.setCentralWidget(central)
        self.statusBar().showMessage(
            f"Nhấn Connect để kết nối {UART_PORT} @ {UART_BAUDRATE} baud"
        )

    def _selected_pre_samples(self) -> int:
        total = FIXED_BUFFER_SIZE
        percentage = self.pre_post_slider.value()
        return int(total * (percentage / 100.0))

    def _selected_post_samples(self) -> int:
        total = FIXED_BUFFER_SIZE
        return total - self._selected_pre_samples()

    def _build_waveform_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        plot_toolbar = QFrame()
        plot_toolbar.setObjectName("panelBar")
        toolbar = QHBoxLayout(plot_toolbar)
        toolbar.setContentsMargins(10, 5, 10, 5)
        title = QLabel("∿  Dạng sóng số")
        title.setObjectName("panelTitle")
        toolbar.addWidget(title)
        toolbar.addStretch(1)

        self.zoom_out_button = QPushButton("−")
        self.zoom_out_button.setFixedWidth(34)
        self.zoom_out_button.clicked.connect(lambda: self._zoom_x(1.35))
        self.zoom_in_button = QPushButton("+")
        self.zoom_in_button.setFixedWidth(34)
        self.zoom_in_button.clicked.connect(lambda: self._zoom_x(1 / 1.35))
        self.fit_button = QPushButton("Vừa bản ghi")
        self.fit_button.clicked.connect(self.fit_waveform)
        toolbar.addWidget(self.zoom_out_button)
        toolbar.addWidget(self.zoom_in_button)
        toolbar.addWidget(self.fit_button)
        layout.addWidget(plot_toolbar)

        self.view_box = HorizontalViewBox(self._mark_manual_zoom)
        self.view_box.setMouseEnabled(x=True, y=False)
        self.plot = pg.PlotWidget(viewBox=self.view_box)
        self.plot.setBackground("#11161d")
        self.plot.showGrid(x=True, y=False, alpha=0.22)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Channel")
        self.plot.getAxis("left").setWidth(55)
        self.plot.getAxis("left").setTextPen("#aab5c3")
        self.plot.getAxis("bottom").setTextPen("#aab5c3")
        self.plot.getAxis("bottom").setPen("#46515f")
        self.plot.getAxis("left").setPen("#46515f")
        self.plot.setMenuEnabled(False)
        self.view_box.setLimits(xMin=0, yMin=-0.3, yMax=12.1)
        self.view_box.setYRange(-0.25, 12.0, padding=0)
        ticks = []
        for channel in reversed(range(8)):
            base = (7 - channel) * 1.5
            ticks.append((base + 0.34, f"D{channel}"))
        self.plot.getAxis("left").setTicks([ticks])

        # Khởi tạo sẵn 8 đường curve không kèm stepMode trên mảng rỗng
        for _ in range(8):
            curve = self.plot.plot(pen=pg.mkPen("#36d17d", width=1.8))
            curve.setVisible(False)
            self.curves.append(curve)

        self.placeholder = pg.TextItem(
            f"Chưa có dữ liệu\nNhấn Connect {UART_PORT} rồi nhấn RUN để vẽ tức thì",
            color="#758294",
            anchor=(0.5, 0.5),
        )
        self.placeholder.setPos(0, 5.8)
        self.plot.addItem(self.placeholder)

        layout.addWidget(self.plot, 1)

        info = QFrame()
        info.setObjectName("panelBar")
        info_layout = QHBoxLayout(info)
        info_layout.setContentsMargins(10, 7, 10, 7)
        self.sample_count_label = QLabel("0 mẫu")
        self.sample_count_label.setObjectName("panelTitle")
        self.trigger_state_label = QLabel("Trigger: chưa kích hoạt")
        self.trigger_state_label.setObjectName("muted")
        self.rate_info_label = QLabel("Δt: —")
        self.rate_info_label.setObjectName("muted")
        info_layout.addWidget(self.sample_count_label)
        info_layout.addSpacing(15)
        info_layout.addWidget(self.trigger_state_label)
        info_layout.addStretch(1)
        info_layout.addWidget(self.rate_info_label)
        layout.addWidget(info)

        return panel

    def _build_decoder_panel(self) -> QWidget:
        """A read-only 8-channel result table; decoding is fully automatic."""
        panel = QGroupBox("GIẢI MÃ TỰ ĐỘNG THEO KÊNH")
        panel.setMinimumHeight(250)
        outer = QVBoxLayout(panel)
        outer.setContentsMargins(10, 14, 10, 8)
        outer.setSpacing(4)

        self.decoder_table = QTableWidget(8, 3)
        self.decoder_table.setHorizontalHeaderLabels(
            ["Kênh", "Dự đoán giao thức", "Data"]
        )
        self.decoder_table.verticalHeader().setVisible(False)
        self.decoder_table.setAlternatingRowColors(True)
        self.decoder_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.decoder_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.decoder_table.setWordWrap(False)
        self.decoder_table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.decoder_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.decoder_table.setMinimumHeight(220)
        self.decoder_table.setSortingEnabled(False)

        header = self.decoder_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.decoder_table.setColumnWidth(0, 72)
        self.decoder_table.setColumnWidth(1, 220)

        self.decoder_row_cache = []
        for channel in range(8):
            self.decoder_table.setRowHeight(channel, 26)
            self.decoder_table.setItem(channel, 0, QTableWidgetItem(f"D{channel}"))
            self.decoder_table.setItem(channel, 1, QTableWidgetItem("Chưa có dữ liệu"))
            self.decoder_table.setItem(channel, 2, QTableWidgetItem("—"))
            self.decoder_table.item(channel, 0).setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.decoder_row_cache.append({
                "protocol": "Chưa có dữ liệu",
                "bit": "—",
                "tooltip": "Nhấn RUN để thu và tự động phân tích kênh này.",
            })

        outer.addWidget(self.decoder_table, 1)
        return panel

    @staticmethod
    def _decoded_values_to_bits(values: list[int], max_bytes: int = 24) -> str:
        """Format decoded protocol payload bytes as a readable bit stream."""
        if not values:
            return "—"
        shown = values[:max_bytes]
        text = " ".join(f"{value & 0xFF:08b}" for value in shown)
        if len(values) > len(shown):
            text += f" … (+{len(values) - len(shown)} byte)"
        return text

    @staticmethod
    def _raw_channel_bits(samples: np.ndarray, channel: int, limit: int = 72) -> str:
        bits = _bits_for_channel(samples, channel)
        if bits.size == 0:
            return "—"
        shown = "".join(str(int(bit)) for bit in bits[:limit])
        if bits.size > limit:
            shown += f" … (+{bits.size - limit} mẫu)"
        return shown

    def _render_decoder_rows(self) -> None:
        if not hasattr(self, "decoder_table"):
            return
        for channel in range(8):
            record = self.decoder_row_cache[channel] if channel < len(self.decoder_row_cache) else {}
            values = (
                f"D{channel}",
                record.get("protocol", "Chưa nhận diện"),
                record.get("bit", "—"),
            )
            for column, value in enumerate(values):
                item = self.decoder_table.item(channel, column)
                if item is None:
                    item = QTableWidgetItem()
                    self.decoder_table.setItem(channel, column, item)
                item.setText(value)
                item.setToolTip(record.get("tooltip", ""))
                if column == 0:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

    def _reset_decoder_table(self) -> None:
        self.decoder_row_cache = [
            {
                "protocol": "Chưa có dữ liệu",
                "bit": "—",
                "tooltip": "Nhấn RUN để thu và tự động phân tích kênh này.",
            }
            for _ in range(8)
        ]
        self._render_decoder_rows()

    def auto_detect_protocol(self, *_args) -> None:
        """Automatically identify and decode protocol activity for each channel.

        The routine does not require the user to select a protocol or pin map.
        It tests shared I2C/SPI line pairs first, then UART per pin and PWM on any
        remaining pin. Unclassified channels are shown as raw sampled logic.
        Because protocol inference is heuristic, ambiguous signals remain
        marked as unclassified rather than being presented as certain.
        """
        if not self.capture_data:
            self._reset_decoder_table()
            return

        samples = np.frombuffer(bytes(self.capture_data), dtype=np.uint8)
        fs = float(self.sample_rate)
        records: list[dict[str, str]] = []
        for channel in range(8):
            transitions = int(np.count_nonzero(
                _bits_for_channel(samples, channel)[1:] !=
                _bits_for_channel(samples, channel)[:-1]
            )) if samples.size > 1 else 0
            records.append({
                "protocol": "Tín hiệu số / chưa nhận diện" if transitions else "Mức logic cố định",
                "bit": self._raw_channel_bits(samples, channel),
                "tooltip": (
                    f"Dữ liệu lấy mẫu thô từ D{channel}; {transitions:,} lần chuyển mức. "
                    "Chuỗi BIT thể hiện trạng thái tại từng thời điểm lấy mẫu, không phải payload đã giải mã."
                ),
            })

        assigned: set[int] = set()

        # Identify highly periodic pulse trains before UART scanning. A stable
        # PWM waveform can otherwise look like several short UART frames when
        # its falling/rising edges are sampled at a coincidental baud rate.
        stable_pwm_channels: set[int] = set()
        for channel in range(8):
            line = _bits_for_channel(samples, channel)
            rising = np.flatnonzero((line[1:] == 1) & (line[:-1] == 0)) + 1
            falling = np.flatnonzero((line[1:] == 0) & (line[:-1] == 1)) + 1
            if rising.size >= 4:
                periods = np.diff(rising).astype(float)
                period_cv = float(np.std(periods) / np.mean(periods)) if np.mean(periods) else 1.0
                duties = []
                for index in range(len(rising) - 1):
                    start = int(rising[index])
                    next_rise = int(rising[index + 1])
                    falls = falling[(falling > start) & (falling < next_rise)]
                    if falls.size:
                        duties.append((int(falls[0]) - start) / (next_rise - start))
                duty_cv = float(np.std(duties) / np.mean(duties)) if duties and np.mean(duties) else 1.0
                if period_cv <= 0.04 and len(duties) >= 2 and duty_cv <= 0.08:
                    stable_pwm_channels.add(channel)

        # 2) I2C: find SCL/SDA pairs with at least one decoded address event.
        # Do not re-use channels already strongly identified as UART.
        i2c_candidates: list[tuple[float, int, int, list[dict]]] = []
        for scl in range(8):
            if scl in assigned:
                continue
            for sda in range(8):
                if sda == scl or sda in assigned:
                    continue
                events, _ = decode_i2c(samples, fs, scl, sda)
                address_events = [
                    event for event in events
                    if event.get("data", "").startswith("ADDR ")
                    and event.get("details", "").endswith("· ACK")
                ]
                if address_events:
                    score = len(address_events) * 100.0 + len(events)
                    i2c_candidates.append((score, scl, sda, events))

        used_i2c: set[int] = set()
        for score, scl, sda, events in sorted(i2c_candidates, reverse=True, key=lambda item: item[0]):
            if scl in assigned or sda in assigned or scl in used_i2c or sda in used_i2c:
                continue
            byte_values = [
                int(match, 16)
                for event in events
                for match in re.findall(r"0x([0-9A-Fa-f]{2})", event.get("data", ""))
            ]
            records[scl] = {
                "protocol": "I²C · SCL",
                "bit": "Clock",
                "tooltip": f"Đường clock I²C SCL; ghép với SDA=D{sda}.",
            }
            records[sda] = {
                "protocol": "I²C · SDA",
                "bit": self._decoded_values_to_bits(byte_values),
                "tooltip": (
                    f"I²C SDA; SCL=D{scl}; {len(events)} event. "
                    "Data gồm địa chỉ 7-bit và các byte dữ liệu, hiển thị theo dạng 8-bit."
                ),
            }
            used_i2c.update((scl, sda))
            assigned.update((scl, sda))

        # 3) SPI: search clock/data pairs and all four CPOL/CPHA modes.
        # One complete byte is accepted, but candidates on already assigned
        # UART/I2C channels are ignored to avoid overwriting stronger matches.
        spi_candidates: list[tuple[float, int, int, int, int, list[dict]]] = []
        for sclk in range(8):
            if sclk in assigned:
                continue
            for mosi in range(8):
                if mosi == sclk or mosi in assigned:
                    continue
                # A clock-like PWM signal paired with a constant pin can
                # otherwise decode as all-zero/all-one SPI bytes. Require
                # actual transitions on the candidate data line.
                mosi_line = _bits_for_channel(samples, mosi)
                mosi_transitions = int(np.count_nonzero(mosi_line[1:] != mosi_line[:-1]))
                if mosi_transitions < 2:
                    continue
                for cpol in (0, 1):
                    for cpha in (0, 1):
                        events, _ = decode_spi(
                            samples, fs, sclk, mosi, -1, -1, cpol, cpha, True
                        )
                        if events:
                            # Prefer multiple decoded bytes. Give a small bonus
                            # when clock edges are regular and evenly spaced.
                            clock = _bits_for_channel(samples, sclk)
                            edge_idx = np.flatnonzero(clock[1:] != clock[:-1]) + 1
                            regularity = 0.0
                            if edge_idx.size >= 4:
                                gaps = np.diff(edge_idx).astype(float)
                                mean_gap = float(np.mean(gaps))
                                if mean_gap > 0:
                                    cv = float(np.std(gaps) / mean_gap)
                                    regularity = max(0.0, 10.0 * (1.0 - min(cv, 1.0)))
                            score = len(events) * 100.0 + regularity
                            spi_candidates.append((score, sclk, mosi, cpol, cpha, events))

        used_spi: set[int] = set()
        for score, sclk, mosi, cpol, cpha, events in sorted(
            spi_candidates, reverse=True, key=lambda item: item[0]
        ):
            if sclk in assigned or mosi in assigned or sclk in used_spi or mosi in used_spi:
                continue
            tx_values = [
                int(match, 16)
                for event in events
                for match in re.findall(r"MOSI=0x([0-9A-Fa-f]{2})", event.get("data", ""))
            ]
            records[sclk] = {
                "protocol": f"SPI · SCLK (Mode {cpol}{cpha})",
                "bit": "Clock",
                "tooltip": f"SPI clock; MOSI=D{mosi}; CPOL={cpol}, CPHA={cpha}.",
            }
            records[mosi] = {
                "protocol": f"SPI · MOSI (Mode {cpol}{cpha})",
                "bit": self._decoded_values_to_bits(tx_values),
                "tooltip": (
                    f"SPI MOSI; SCLK=D{sclk}; {len(tx_values)} byte giải mã, MSB first. "
                    "MISO/CS không được tự gán nếu chưa có bằng chứng đủ tin cậy."
                ),
            }
            used_spi.update((sclk, mosi))
            assigned.update((sclk, mosi))

        # 4) UART: inspect each channel independently and select its strongest
        # valid 8N1 candidate. A valid HIGH stop-bit is required. Stable pulse
        # trains are held back for PWM/SPI classification to reduce false UARTs.
        baud_candidates = (9600, 19200, 38400, 57600, 115200, 230400, 460800)
        for channel in range(8):
            if channel in stable_pwm_channels or channel in assigned:
                continue
            best = None
            for baud in baud_candidates:
                events, _ = decode_uart(samples, fs, channel, baud)
                valid_events = [event for event in events if "Stop OK" in event.get("details", "")]
                if not valid_events:
                    continue
                # When a true UART frame is oversampled, decoding it at a
                # multiple baud can create false short frames from transitions
                # inside the payload. Prefer the slowest candidate that still
                # has a valid stop bit; frame count is only a small tie-breaker.
                samples_per_bit = fs / float(baud)
                score = samples_per_bit * 10.0 + min(len(valid_events), 100) * 0.01
                if best is None or score > best[0]:
                    values = [
                        int(match, 16)
                        for event in valid_events
                        for match in re.findall(r"0x([0-9A-Fa-f]{2})", event.get("data", ""))
                    ]
                    best = (score, baud, valid_events, values)
            if best is not None:
                _, baud, events, values = best
                records[channel] = {
                    "protocol": f"UART RX · {baud:,} baud",
                    "bit": self._decoded_values_to_bits(values),
                    "tooltip": (
                        f"UART asynchronous 8N1; {len(values)} byte đã giải mã. "
                        "Mỗi byte hiển thị MSB trước; kiểm tra baud/parity nếu dữ liệu không như mong đợi."
                    ),
                }
                assigned.add(channel)


        # 5) PWM: identify repeated rising-edge periods on channels not already
        # consumed by a stronger serial-protocol candidate.
        for channel in range(8):
            if channel in assigned:
                continue
            events, _ = decode_pwm(samples, fs, channel)
            if len(events) >= 2 and channel in stable_pwm_channels:
                first_events = events[:8]
                measurements = []
                for event in first_events:
                    measurements.append(f"{event['data']} · {event['details'].split(' · ')[-1]}")
                records[channel] = {
                    "protocol": "PWM",
                    "bit": " | ".join(measurements),
                    "tooltip": (
                        f"PWM đo trên D{channel}; {len(events)} chu kỳ. "
                        "Data hiển thị tần số và duty cycle đo được."
                    ),
                }
                assigned.add(channel)

        self.decoder_row_cache = records
        self._render_decoder_rows()
        detected = sum(1 for row in records if row["protocol"].startswith(("UART", "SPI", "I²C", "PWM")))
        self.statusBar().showMessage(
            f"Đã tự phân tích 8 kênh · nhận diện giao thức trên {detected} kênh · "
            f"{len(samples):,} mẫu"
        )

    def _apply_style(self) -> None:
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background: #171c23;
                color: #d7dee8;
                font-family: 'Segoe UI', 'Arial';
                font-size: 10pt;
            }
            QLabel#brand { font-size: 17pt; font-weight: 700; color: #e6edf7; }
            QLabel#muted { color: #8d99a9; }
            QLabel#statusPill {
                color: #e0a84b; background: #302a20;
                border: 1px solid #665332; border-radius: 10px;
                padding: 5px 10px; font-weight: 600;
            }
            QFrame#actionBar, QFrame#panelBar {
                background: #1d242d; border: 1px solid #303a47; border-radius: 6px;
            }
            QLabel#panelTitle { color: #e3eaf4; font-weight: 600; }
            QLabel#fieldLabel { color: #aab6c5; font-size: 9pt; padding-top: 2px; }
            QLabel#inlineFieldLabel { color: #aab6c5; font-size: 9pt; padding-left: 3px; padding-right: 1px; }
            QLabel#sliderValue { color: #8bb6df; font-weight: 700; font-size: 8pt; }
            QGroupBox {
                border: 1px solid #303a47; border-radius: 6px;
                margin-top: 10px; padding: 10px 8px 8px 8px;
                color: #9eabbc; font-weight: 600;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 9px; padding: 0 5px; }
            QComboBox {
                background: #11161d; border: 1px solid #394553;
                border-radius: 4px; padding: 4px 6px; min-height: 20px;
                color: #dce4ee;
            }
            QComboBox::drop-down { border: 0; width: 22px; }
            QComboBox QAbstractItemView {
                background: #202833; color: #dce4ee;
                selection-background-color: #38516c; border: 1px solid #455366;
            }
            QTableWidget {
                background: #11161d; alternate-background-color: #1a222c;
                color: #d7dee8; gridline-color: #2d3744;
                border: 1px solid #303a47; selection-background-color: #304a63;
            }
            QHeaderView::section {
                background: #202833; color: #aab6c5;
                border: 0; border-bottom: 1px solid #394553;
                padding: 5px 7px; font-weight: 600;
            }
            QSlider#sampleSlider { min-height: 25px; }
            QSlider#sampleSlider::groove:horizontal {
                height: 5px; background: #303b49; border-radius: 2px;
            }
            QSlider#sampleSlider::handle:horizontal {
                width: 15px; margin: -6px 0; border-radius: 7px;
                background: #8bb6df; border: 1px solid #b2d2ef;
            }
            QSlider#sampleSlider::sub-page:horizontal {
                background: #547fa8; border-radius: 2px;
            }
            QPushButton {
                background: #293340; border: 1px solid #435164;
                border-radius: 4px; padding: 4px 7px; color: #dce4ee;
            }
            QPushButton:hover { background: #344253; }
            QPushButton:disabled { color: #657181; background: #222a34; border-color: #303945; }
            QPushButton#connectButton {
                background: #1f3b31; border: 1px solid #3c8c6c;
                color: #7ee2b8; font-weight: 700; min-width: 72px;
            }
            QPushButton#connectButton:hover { background: #274a3e; }
            QPushButton#runButton {
                background: #8bb6df; border: 1px solid #a4c9ed;
                color: #101820; font-weight: 700; min-width: 54px;
            }
            QPushButton#runButton:hover { background: #a4c8eb; }
            QPushButton#stopButton {
                background: #38282b; border: 1px solid #70444b;
                color: #f0a1a8; font-weight: 600; min-width: 56px;
            }
            QStatusBar { color: #8d99a9; background: #141920; border-top: 1px solid #303a47; }
        """)

    # ----- Device connection --------------------------------------------------

    def _close_serial(self) -> None:
        self.poll_timer.stop()
        self.watchdog_timer.stop()
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except (serial.SerialException, OSError):
                pass
            finally:
                self.ser = None

    def _handle_disconnect(self, reason: str) -> None:
        self._close_serial()
        if self.acquiring:
            self.acquiring = False
            self._set_running_ui(False)
            self.capture_label.setText(f"Mất kết nối {UART_PORT} · {self.total_received:,} mẫu")
            self.trigger_state_label.setText("Trigger: ngắt do mất kết nối")
        self._set_status(f"DISCONNECTED · {UART_PORT}", "#ef626f")
        self.statusBar().showMessage(f"{reason} — Cắm lại cáp và nhấn 'Connect {UART_PORT}' để kết nối lại.")

    def _check_port_alive(self) -> None:
        if self.ser is None:
            return
        available_ports = {p.device for p in list_ports.comports()}
        if UART_PORT not in available_ports:
            self._handle_disconnect(f"{UART_PORT} đã bị rút ra khỏi máy tính")

    def connect_uart(self) -> bool:
        if self.acquiring:
            self.stop_acquisition(send_command=False)

        self._close_serial()

        try:
            self.ser = serial.Serial(
                port=UART_PORT,
                baudrate=UART_BAUDRATE,
                timeout=0,
                write_timeout=0.5,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.poll_timer.start()
            self.watchdog_timer.start()
            self._set_status(f"CONNECTED · {UART_PORT}", "#50c78b")
            self.statusBar().showMessage(f"Đã kết nối {UART_PORT} @ {UART_BAUDRATE} baud")
            return True
        except (serial.SerialException, OSError) as exc:
            self.ser = None
            self._set_status(f"DISCONNECTED · {UART_PORT}", "#ef626f")
            self.statusBar().showMessage(f"Không thể kết nối {UART_PORT}: {exc}")
            QMessageBox.critical(
                self,
                f"Không thể kết nối {UART_PORT}",
                f"Không mở được cổng {UART_PORT}.\n"
                f"Hãy kiểm tra cáp USB-UART đã cắm chắc chắn chưa hoặc đóng phần mềm khác đang chiếm {UART_PORT}.\n\n"
                f"Chi tiết: {exc}",
            )
            return False

    def _set_status(self, text: str, color: str) -> None:
        self.top_status.setText(f"● {text}")
        self.top_status.setStyleSheet(
            f"color:{color}; background:#252d37; border:1px solid #3b4654; "
            "border-radius:10px; padding:5px 10px; font-weight:600;"
        )

    def _rearm_fpga(self) -> None:
        if not (self.ser and self.ser.is_open and self.acquiring):
            return
        try:
            self.ser.write(build_command(CMD_STOP))
            self.ser.write(build_command(CMD_RUN))
            self.ser.flush()
            self.batch_received = 0
        except (serial.SerialException, OSError) as exc:
            self._handle_disconnect(f"Mất kết nối {UART_PORT} khi kích lại lấy mẫu ({exc})")

    def _poll_serial(self) -> None:
        if not self.ser or not self.ser.is_open:
            return
        try:
            available = self.ser.in_waiting
            chunk = self.ser.read(min(available, 65536)) if available > 0 else b""
        except (serial.SerialException, OSError) as exc:
            self._handle_disconnect(f"Mất kết nối {UART_PORT} ({exc})")
            return

        if not self.acquiring:
            return

        if chunk:
            self.total_uart_bytes += len(chunk)
            self.has_new_data = False
            if self.protocol_mode == "timestamp_data":
                self._consume_timestamp_data_pairs(chunk)
            elif self.protocol_mode == "framed":
                self.parser.feed(chunk)
            elif self.protocol_mode == "raw":
                # Optional mode for hardware that sends one sample byte per UART byte.
                self._append_samples(chunk)

            # Cứ có dữ liệu mới là vẽ lên đồ thị ngay lập tức, không chờ
            if self.has_new_data:
                self._refresh_plot()
        # No re-arm loop here: once the selected sample count is reached,
        # _append_samples schedules a STOP command automatically.

    # ----- Acquisition --------------------------------------------------------

    def start_acquisition(self) -> None:
        if not (self.ser and self.ser.is_open):
            QMessageBox.information(
                self,
                "Chưa kết nối UART",
                f"Vui lòng nhấn nút 'Connect {UART_PORT}' trước khi bấm RUN.",
            )
            return

        if self.acquiring:
            return

        self.sample_rate = int(self.rate_combo.currentData())
        # converter.v sends two UART bytes per 16-bit word: timestamp [15:8],
        # followed by logic sample DATA [7:0]. Do not use the old 5A A5 parser.
        self.protocol_mode = "timestamp_data"
        self.rx_pair_buffer.clear()
        self.total_uart_bytes = 0
        self.total_pairs_received = 0
        trigger_channel = int(self.trigger_channel_combo.currentData())
        trigger_mode = int(self.trigger_mode_combo.currentData())

        pre_count = self._selected_pre_samples()
        post_count = self._selected_post_samples()
        self.target_sample_count = max(1, pre_count + post_count)
        self.batch_size = self.target_sample_count
        self.batch_received = 0
        self.total_received = 0
        self.last_rx_time = 0.0
        self.auto_stop_scheduled = False

        self.parser.reset()
        self.raw_capture = None
        self.auto_follow = True
        self._clear_waveform()

        try:
            self.ser.reset_input_buffer()
            config = build_config_frame(
                self.sample_rate, trigger_channel, trigger_mode, pre_count, post_count
            )
            self.ser.write(config)
            self.ser.write(build_command(CMD_RUN))
            self.ser.flush()
        except (serial.SerialException, OSError) as exc:
            self._handle_disconnect(f"Lỗi ghi lệnh xuống {UART_PORT}")
            QMessageBox.critical(
                self,
                "Lỗi UART",
                f"Không gửi được lệnh RUN qua {UART_PORT} (cáp có thể đã bị rút).\n"
                f"Hãy cắm lại cáp và nhấn 'Connect {UART_PORT}'.\n\nChi tiết: {exc}",
            )
            return

        self.acquiring = True
        self._set_running_ui(True)
        self.trigger_state_label.setText("Trigger: đang thu & vẽ trực tiếp")
        self.capture_label.setText("Đang chờ UART: mỗi mẫu gồm Timestamp + DATA")
        self.statusBar().showMessage(
            f"Đang nhận cặp [Timestamp][DATA] @ {UART_BAUDRATE} baud · "
            f"Tự dừng khi đủ {self.target_sample_count:,} mẫu"
        )

    def _auto_stop_if_complete(self) -> None:
        self.auto_stop_scheduled = False
        if self.acquiring and self.total_received >= self.target_sample_count:
            self.stop_acquisition(send_command=True, automatic=True)

    def stop_acquisition(self, send_command: bool = True, automatic: bool = False) -> None:
        was_acquiring = self.acquiring
        if send_command and self.ser and self.ser.is_open and was_acquiring:
            try:
                self.ser.write(build_command(CMD_STOP))
                self.ser.flush()
            except (serial.SerialException, OSError):
                self._handle_disconnect(f"Mất kết nối {UART_PORT} khi dừng")
                return
        self.acquiring = False
        self._set_running_ui(False)
        if was_acquiring:
            tail_note = " · còn 1 byte lẻ chưa đủ cặp" if self.rx_pair_buffer else ""
            if automatic:
                stop_text = f"Đã tự dừng · đủ {self.total_received:,}/{self.target_sample_count:,} mẫu"
            else:
                stop_text = f"Đã dừng thủ công · {self.total_received:,}/{self.target_sample_count:,} mẫu"
            self.capture_label.setText(
                f"{stop_text} · {self.total_uart_bytes:,} byte UART{tail_note}"
            )
            self.trigger_state_label.setText("Trigger: đã dừng")
            self._refresh_plot()
            self.statusBar().showMessage(stop_text)
            if self.total_received:
                # After a completed capture, run the protocol heuristic and
                # populate the per-channel 3-column results table.
                self.auto_detect_protocol()

    def _set_running_ui(self, running: bool) -> None:
        self.run_button.setEnabled(not running)
        self.stop_button.setEnabled(running)
        for widget in (
            self.rate_combo, self.pre_post_slider,
            self.trigger_channel_combo, self.trigger_mode_combo,
        ):
            widget.setEnabled(not running)

    # ----- Receive callbacks --------------------------------------------------

    def _consume_timestamp_data_pairs(self, chunk: bytes) -> None:
        """Decode converter.v stream: [timestamp byte][DATA byte] per sample.

        converter.v sends saved_data[15:8] first, then saved_data[7:0].
        pyserial read boundaries are arbitrary, so an unfinished byte pair is
        retained and completed by the next read(). Only DATA bytes are plotted.
        """
        if not chunk:
            return

        self.rx_pair_buffer.extend(chunk)
        pair_count = len(self.rx_pair_buffer) // 2
        if pair_count == 0:
            self.capture_label.setText(
                f"Đã nhận {self.total_uart_bytes:,} byte UART · đang chờ byte DATA"
            )
            return

        complete_length = pair_count * 2
        # Pair format: [timestamp0, data0, timestamp1, data1, ...]
        complete_pairs = self.rx_pair_buffer[:complete_length]
        del self.rx_pair_buffer[:complete_length]

        # Keep timestamp bytes available for future diagnostics; waveform x-axis
        # uses the configured sample rate because timestamp units are not defined
        # by converter.v itself.
        data_samples = bytes(complete_pairs[1::2])
        self.total_pairs_received += pair_count
        if data_samples:
            self._append_samples(data_samples)

    def _append_samples(self, payload: bytes) -> None:
        if not payload or self.total_received >= self.target_sample_count:
            return

        remaining = self.target_sample_count - self.total_received
        accepted = payload[:remaining]
        if not accepted:
            return

        self.capture_data.extend(accepted)
        overflow = len(self.capture_data) - MAX_DISPLAY_SAMPLES
        if overflow > 0:
            del self.capture_data[:overflow]

        n = len(accepted)
        self.total_received += n
        self.batch_received += n
        self.last_rx_time = time.monotonic()
        self.has_new_data = True
        self._update_capture_labels()

        if (self.acquiring and self.total_received >= self.target_sample_count
                and not self.auto_stop_scheduled):
            self.auto_stop_scheduled = True
            QTimer.singleShot(0, self._auto_stop_if_complete)

    def _on_framed_data(self, payload: bytes) -> None:
        if not self.acquiring or not payload:
            return
        self._append_samples(payload)

    def _on_device_status(self, status: int) -> None:
        if status == STATUS_COMPLETE and self.acquiring:
            self._rearm_fpga()
        elif status == 0:
            self.trigger_state_label.setText("Trigger: FPGA sẵn sàng")

    def _update_capture_labels(self) -> None:
        shown = len(self.capture_data)
        self.sample_count_label.setText(f"{shown:,} mẫu (tổng {self.total_received:,})")
        if self.acquiring:
            self.capture_label.setText(
                f"UART RX: {self.total_uart_bytes:,} byte · "
                f"{self.total_received:,} mẫu DATA"
            )
        if self.sample_rate:
            dt_ns = 1e9 / self.sample_rate
            self.rate_info_label.setText(f"Δt: {dt_ns:g} ns / mẫu")

    # ----- Plot rendering -----------------------------------------------------

    def _clear_waveform(self) -> None:
        self.capture_data.clear()
        self.rx_pair_buffer.clear()
        self.placeholder.setVisible(True)
        for curve in self.curves:
            curve.setData([], [])
            curve.setVisible(False)
        self.has_new_data = False
        if hasattr(self, "decoder_table"):
            self._reset_decoder_table()
        self.sample_count_label.setText("0 mẫu")
        self.rate_info_label.setText(f"Δt: {1e9 / self.sample_rate:g} ns / mẫu")
        self.view_box.setXRange(0, 1e-5, padding=0)

    def _refresh_plot(self, *_args) -> None:
        """Render each DATA byte as a digital sample on D0..D7.

        Build explicit horizontal and vertical segments instead of relying on
        PlotDataItem(stepMode=...). This works consistently across pyqtgraph
        versions and avoids the x/y length convention of stepMode.
        """
        count = len(self.capture_data)
        if count == 0:
            self.placeholder.setVisible(True)
            for curve in self.curves:
                curve.setData([], [])
                curve.setVisible(False)
            return

        # Each byte is one simultaneous sample of D0..D7.
        samples = np.frombuffer(bytes(self.capture_data), dtype=np.uint8)
        if samples.size != count:
            self.statusBar().showMessage(
                f"Lỗi dữ liệu waveform: nhận {samples.size} byte, mong đợi {count}"
            )
            return

        dt_s = 1.0 / max(1.0, float(self.sample_rate))
        x_edges = np.arange(count + 1, dtype=np.float64) * dt_s
        self.placeholder.setVisible(False)

        if self.auto_follow:
            xmax = max(dt_s * count, dt_s)
            self.view_box.setXRange(0.0, xmax, padding=0.015)

        visible_channels = 0
        try:
            for channel in range(8):
                curve = self.curves[channel]
                bits = ((samples >> channel) & 1).astype(np.float64)
                transitions = (
                    int(np.count_nonzero(bits[1:] != bits[:-1])) if count > 1 else 0
                )
                pen_color = "#36d17d" if transitions > 0 else "#ef5965"

                # Give each channel its own horizontal lane. A low bit is at
                # lane_base, a high bit is lane_base + 0.68.
                lane_base = (7 - channel) * 1.5
                levels = lane_base + bits * 0.68

                # Explicit step trace, with equal-length X/Y arrays:
                # (t0,y0)->(t1,y0)->(t1,y1)->(t2,y1)...
                # This draws horizontal holds and vertical transitions.
                x_plot = np.empty(count * 2, dtype=np.float64)
                y_plot = np.empty(count * 2, dtype=np.float64)
                x_plot[0::2] = x_edges[:-1]
                x_plot[1::2] = x_edges[1:]
                y_plot[0::2] = levels
                y_plot[1::2] = levels

                curve.setPen(pg.mkPen(pen_color, width=1.8))
                curve.setData(x=x_plot, y=y_plot)
                curve.setVisible(True)
                visible_channels += 1

            self.plot.update()
            self.statusBar().showMessage(
                f"Đang hiển thị {count:,} mẫu DATA trên {visible_channels}/8 kênh"
            )
        except Exception as exc:
            # Surface rendering errors instead of leaving an empty plot with
            # no indication of what failed.
            self.statusBar().showMessage(f"Lỗi vẽ waveform: {type(exc).__name__}: {exc}")
            print(f"[LogicAnalyzer] waveform rendering failed: {exc!r}")

    def _mark_manual_zoom(self) -> None:
        self.auto_follow = False

    def _zoom_x(self, factor: float) -> None:
        (xmin, xmax), _ = self.view_box.viewRange()
        center = max(0.0, (xmin + xmax) / 2)
        width = max((xmax - xmin) * factor, 1e-12)
        new_min = max(0.0, center - width / 2)
        new_max = max(new_min + 1e-12, center + width / 2)
        self.auto_follow = False
        self.view_box.setXRange(new_min, new_max, padding=0)

    def fit_waveform(self) -> None:
        self.auto_follow = True
        count = len(self.capture_data)
        if count:
            duration = count / float(self.sample_rate)
            self.view_box.setXRange(0, max(duration, 1e-12), padding=0.015)
        else:
            self.view_box.setXRange(0, 1e-5, padding=0)

    def closeEvent(self, event):
        self.stop_acquisition(send_command=True)
        self._close_serial()
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("MACH Logic Analyzer")
    app.setFont(QFont("Segoe UI", 9))
    pg.setConfigOptions(antialias=False, foreground="#cbd5e1")
    window = LogicAnalyzerWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())