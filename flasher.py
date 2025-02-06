import argparse
import time
import serial


def crc211(buf):
    crc = 211
    for byte in buf:
        crc += byte * 211
        crc &= 0xFFFF
        crc ^= crc >> 8
    return crc & 0xFF


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


parser = argparse.ArgumentParser(description="Upload firmware to target")
parser.add_argument("file", help="firmware file")
parser.add_argument("--port", "-p", help="serial port", default="COM3")
parser.add_argument("--baud", "-b", help="baud rate", default=115200, type=int)
parser.add_argument("--start", "-s", help="start page", default=1, type=int)
parser.add_argument("--modbus", "-m", help="ModBus", action="store_true")
parser.add_argument("--mAdr", "-ma", help="ModBus address", default=32, type=int)
parser.add_argument("--mBaud", "-mb", help="ModBus baud rate", default=9600, type=int)
parser.add_argument("--mReg", "-mr", help="ModBus register", default=65535, type=int)
parser.add_argument("--mVal", "-mv", help="ModBus value", default=0xDEAD, type=int)
parser.add_argument("--resetCommand", "-rc", help="reset command", default="")
args = parser.parse_args()

MAX_RETRIES = 10


def send_firmware(file, ser):
    with open(file, "rb") as f:
        data = f.read()

        data_list = [data[i : i + 1024] for i in range(0, len(data), 1024)]
        if len(data_list[-1]) < 1024:
            data_list[-1] += b"\xff" * (1024 - len(data_list[-1]))

        for _ in range(MAX_RETRIES):
            if args.modbus:
                ser.apply_settings(
                    {
                        "baudrate": args.mBaud,
                        "parity": serial.PARITY_NONE,
                        "stopbits": serial.STOPBITS_TWO,
                        "bytesize": serial.EIGHTBITS,
                    }
                )
                msg = (
                    args.mAdr.to_bytes(1, "big")
                    + b"\x06"
                    + args.mReg.to_bytes(2, "big")
                    + args.mVal.to_bytes(2, "big")
                )
                msg += crc16(msg).to_bytes(2, "little")
                ser.write(msg)
                time.sleep(8 * 11 / args.mBaud)

            ser.apply_settings({"baudrate": args.baud})

            if args.resetCommand:
                ser.write(bytes.fromhex(args.resetCommand))

            ser.timeout = 0.01 + (4 * 11 / args.baud)
            while True:
                resp = ser.read(4)
                if resp == b"\xfe\xe1\xde\xad":
                    break

            ser.timeout = 0.1 + (1030 * 11 / args.baud)
            for i, page in enumerate(reversed(data_list)):
                print(f"Writing page {i + 1}/{len(data_list)}")
                packet = bytearray()
                packet.extend(b"\xde\xad\xbe\xef")
                packet.append(len(data_list) - i - 1 + args.start)
                packet.append(crc211(page))
                packet.extend(page)
                ser.write(packet)
                resp = ser.read(1)
                if resp != b"\xAA":
                    print("Didn't get ACK, retrying...")
                    break
            else:
                print("Firmware upload successful!")
                return

            ser.reset_input_buffer()
            ser.reset_output_buffer()

        print("Max retries reached. Firmware upload unsuccessful.")


try:
    with serial.Serial(args.port, args.baud) as ser:
        send_firmware(args.file, ser)
except serial.SerialException as e:
    print(f"Serial port error: {e}")
except FileNotFoundError:
    print("File not found.")
except Exception as e:
    print(f"An error occurred: {e}")
