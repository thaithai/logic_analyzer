import struct
import time
import serial

PORT = "COM11"
BAUDRATE = 115200

# Cấu hình lấy mẫu gửi xuống FPGA
SAMPLE_RATE = 1_000_000   # 1 MS/s (100_000, 500_000, 1_000_000, 5_000_000, 10_000_000, 25_000_000, 50_000_000)
TRIGGER_CHANNEL = 0       # Kênh D0
TRIGGER_MODE = 0          # 0: Không trigger
PRE_SAMPLES = 0           # Số mẫu trước trigger
POST_SAMPLES = 256        # Số mẫu cần thu

# Giao thức khung truyền UART PC <-> FPGA
HOST_SYNC = b"\xA5\x5A"
DEVICE_SYNC = b"\x5A\xA5"
CMD_CONFIG = 0x01
CMD_RUN = 0x02
CMD_STOP = 0x03
PKT_DATA = 0x10
PKT_STATUS = 0x11
STATUS_COMPLETE = 0x01


def build_config_packet(sample_rate: int, channel: int, mode: int, pre: int, post: int) -> bytes:
    """Đóng gói khung cấu hình 13 byte: A5 5A 01 <rate:4B> <ch:1B> <mode:1B> <pre:2B> <post:2B>."""
    payload = struct.pack("<IBBHH", sample_rate, channel, mode, pre, post)
    return HOST_SYNC + bytes([CMD_CONFIG]) + payload


def main():
    total_expected = PRE_SAMPLES + POST_SAMPLES

    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.5,
        )
    except serial.SerialException as e:
        print(f"Không thể mở cổng {PORT}: {e}")
        return

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 1. Gửi gói cấu hình lấy mẫu & chế độ không trigger xuống FPGA
        cfg_pkt = build_config_packet(
            SAMPLE_RATE, TRIGGER_CHANNEL, TRIGGER_MODE, PRE_SAMPLES, POST_SAMPLES
        )
        run_pkt = HOST_SYNC + bytes([CMD_RUN])

        print(f"Đã mở {PORT} @ {BAUDRATE} baud.")
        print(f"Gửi CONFIG ({len(cfg_pkt)} B): {cfg_pkt.hex(' ').upper()}")
        ser.write(cfg_pkt)
        ser.flush()
        time.sleep(0.01)

        # 2. Gửi lệnh RUN để FPGA bắt đầu thu mẫu và đẩy dữ liệu lên PC
        print(f"Gửi RUN    ({len(run_pkt)} B): {run_pkt.hex(' ').upper()}")
        ser.write(run_pkt)
        ser.flush()

        print(f"\nĐang chờ nhận {total_expected} mẫu từ FPGA...\n")
        print("STT  | HEX  | D7 D6 D5 D4 D3 D2 D1 D0")
        print("-" * 38)

        rx_buf = bytearray()
        samples = bytearray()
        done = False
        last_rx_time = time.time()

        # 3. Nhận dữ liệu từ FPGA, tách khung 5A A5 10 và in từng bit D7..D0
        while not done:
            waiting = ser.in_waiting
            if waiting > 0:
                chunk = ser.read(waiting)
                rx_buf.extend(chunk)
                last_rx_time = time.time()

                while True:
                    sync_idx = rx_buf.find(DEVICE_SYNC)
                    if sync_idx < 0:
                        if rx_buf[-1:] == DEVICE_SYNC[:1]:
                            del rx_buf[:-1]
                        else:
                            rx_buf.clear()
                        break

                    if sync_idx > 0:
                        del rx_buf[:sync_idx]

                    if len(rx_buf) < 5:
                        break

                    pkt_type = rx_buf[2]
                    payload_len = rx_buf[3] | (rx_buf[4] << 8)
                    frame_len = 5 + payload_len

                    if len(rx_buf) < frame_len:
                        break

                    payload = bytes(rx_buf[5:frame_len])
                    del rx_buf[:frame_len]

                    if pkt_type == PKT_DATA:
                        for b in payload:
                            if len(samples) >= total_expected:
                                break
                            idx = len(samples)
                            samples.append(b)
                            bits_str = "  ".join(str((b >> ch) & 1) for ch in range(7, -1, -1))
                            print(f"{idx:04d} | 0x{b:02X} |  {bits_str}")

                        if len(samples) >= total_expected:
                            done = True
                            break

                    elif pkt_type == PKT_STATUS and payload:
                        if payload[0] == STATUS_COMPLETE:
                            done = True
                            break
            else:
                if time.time() - last_rx_time > 3.0:
                    print("\nHết thời gian chờ (3s) mà chưa nhận đủ mẫu từ FPGA.")
                    break
                time.sleep(0.005)

        print("-" * 38)
        print(f"Hoàn tất: đã thu được {len(samples)}/{total_expected} mẫu.")

    except KeyboardInterrupt:
        print("\nĐã dừng thủ công bởi người dùng.")
    finally:
        if ser.is_open:
            try:
                ser.write(HOST_SYNC + bytes([CMD_STOP]))
                ser.flush()
            except serial.SerialException:
                pass
            ser.close()
            print(f"Đã gửi lệnh STOP và đóng cổng {PORT}.")


if __name__ == "__main__":
    main()