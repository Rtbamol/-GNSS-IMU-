import serial
import time
from pathlib import Path
from datetime import datetime

TARGET_PORT = "COM4"
BAUDRATE = 115200

# 日志文件夹：自动创建
LOG_DIR = Path(r"D:\A_A\A_Schedule\date_set\Scarab\gnss_python\gnss_logs")
LOG_DIR.mkdir(parents=True, exist_ok=True)

# 每次运行生成一个新的日志文件
start_time = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = LOG_DIR / f"gnss_data_{start_time}.txt"


def connect_device():
    try:
        print(f"Trying to connect to {TARGET_PORT} at {BAUDRATE} baud...")

        with serial.Serial(
            port=TARGET_PORT,
            baudrate=BAUDRATE,
            timeout=1
        ) as ser, open(LOG_FILE, "a", encoding="utf-8") as log_file:

            print(f"Successfully connected to {TARGET_PORT}")
            print(f"Data will be saved to: {LOG_FILE}")
            print("Press Ctrl+C to exit")

            log_file.write(f"Start time: {datetime.now()}\n")
            log_file.write(f"Port: {TARGET_PORT}\n")
            log_file.write(f"Baudrate: {BAUDRATE}\n")
            log_file.write("=" * 80 + "\n")
            log_file.flush()

            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)

                    # 原始 bytes 打印
                    print(data)

                    # 转成字符串保存
                    text = data.decode("utf-8", errors="ignore")

                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                    # 保存带电脑时间戳的数据
                    log_file.write(f"[{timestamp}] {text}")
                    log_file.flush()

                time.sleep(0.1)

    except PermissionError as e:
        print(f"Permission denied when opening {TARGET_PORT}")
        print("原因通常是 COM 口被其他程序占用。")
        print(f"Original error: {e}")

    except serial.SerialException as e:
        print(f"Serial connection failed: {e}")

    except KeyboardInterrupt:
        print("\nDisconnecting...")
        print(f"Data saved to: {LOG_FILE}")


if __name__ == "__main__":
    connect_device()




