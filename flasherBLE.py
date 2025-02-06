import argparse
from bleak import BleakClient, BleakScanner
import asyncio

def crc211(buf):
    crc = 211
    for byte in buf:
        crc += byte * 211
        crc &= 0xFFFF
        crc ^= crc >> 8
    return crc & 0xFF

parser = argparse.ArgumentParser(description="Upload firmware via BLE")
parser.add_argument("file", help="firmware file")
parser.add_argument("--device", "-d", help="BLE device address")
parser.add_argument("--name", "-n", help="BLE device name")
parser.add_argument("--start", "-s", help="start page", default=1, type=int)
parser.add_argument("--resetCommand", "-rc", help="reset command", default="")
parser.add_argument("--serviceUUID", "-su", help="BLE Service UUID", default="0000ffe0-0000-1000-8000-00805f9b34fb")
parser.add_argument("--charUUID", "-cu", help="BLE Characteristic UUID", default="0000ffe1-0000-1000-8000-00805f9b34fb")
args = parser.parse_args()

MAX_RETRIES_PROG = 5

async def send_firmware(file, client):
    with open(file, "rb") as f:
        data = f.read()

        data_list = [data[i : i + 1024] for i in range(0, len(data), 1024)]
        if len(data_list[-1]) < 1024:
            data_list[-1] += b"\xff" * (1024 - len(data_list[-1]))

        notification_event = asyncio.Event()
        notification_data = None

        def notification_handler(sender, data):
            # print("Received notification:", data)
            nonlocal notification_data
            notification_data = data
            notification_event.set()

        await client.start_notify(args.charUUID, notification_handler)

        for _ in range(MAX_RETRIES_PROG):
            notification_event.clear()
            notification_data = None

            if args.resetCommand:
                await client.write_gatt_char(args.charUUID, bytes.fromhex(args.resetCommand), response=True)

            try:
                await asyncio.wait_for(notification_event.wait(), timeout=1)
                if notification_data == b"\xfe\xe1\xde\xad":
                    print("Bootloader ready")
                else:
                    print(f"Unexpected response: {notification_data}")
                    continue
            except asyncio.TimeoutError:
                print("Failed to get bootloader hello, retrying...")
                continue

            for i, page in enumerate(reversed(data_list)):
                print(f"Writing page {i + 1}/{len(data_list)}")
                packet = bytearray()
                packet.extend(b"\xde\xad\xbe\xef")
                packet.append(len(data_list) - i - 1 + args.start)
                packet.append(crc211(page))
                packet.extend(page)

                notification_event.clear()
                notification_data = None

                packet_list = [packet[i : i + 128] for i in range(0, len(packet), 128)]

                for packet in packet_list:
                    await client.write_gatt_char(args.charUUID, packet, response=False)

                try:
                    await asyncio.wait_for(notification_event.wait(), timeout=1)
                    if notification_data == b"\xaa":
                        print(f"ACK received for page {i + 1}")
                    else:
                        print(f"Unexpected response: {notification_data}, retrying...")
                        break
                except asyncio.TimeoutError:
                    print("Didn't get ACK, retrying...")
                    break

            else:
                print("Firmware upload successful!")
                await client.stop_notify(args.charUUID)
                return

        await client.stop_notify(args.charUUID)
        print("Max retries reached. Firmware upload unsuccessful.")


async def main():
    if args.device:
        device = await BleakScanner.find_device_by_address(args.device)
    elif args.name:
        devices = await BleakScanner.discover()
        device = next((d for d in devices if d.name == args.name), None)
    else:
        print("Please specify either device address or name.")
        return

    if device is None:
        print(f"Device {args.device or args.name} not found.")
        return

    async with BleakClient(device) as client:
        services = client.services

        if args.serviceUUID not in [s.uuid for s in services]:
            print(f"Service UUID {args.serviceUUID} not found.")
            return

        if args.charUUID not in [c.uuid for s in services for c in s.characteristics]:
            print(f"Characteristic UUID {args.charUUID} not found.")
            return

        await send_firmware(args.file, client)


asyncio.run(main())
