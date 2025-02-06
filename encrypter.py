"""
    This program encrypts firmware file with speck cipher
"""

import argparse
from speck import SpeckCipher

BOOTLOADER_BLOCK_SIZE = 1024

KEY_SIZE = 128

CYPHER_BLOCK_SIZE = 64
CYPHER_BLOCK_SIZE_BYTES = CYPHER_BLOCK_SIZE // 8


def hex_to_int(hex_str):
    # remove 0x prefix if present
    hex_str = hex_str[2:] if hex_str.startswith("0x") else hex_str
    return int(hex_str, 16)


def output_file_name(name_str):
    # if no extension is provided, add .strn
    if "." not in name_str:
        return name_str + ".strn"
    return name_str


def main():
    parser = argparse.ArgumentParser(
        description="encrypt firmware file with speck cipher"
    )
    parser.add_argument("-i", "--input", help="Firmware file to encrypt", required=True)
    parser.add_argument(
        "-k",
        "--key",
        help="Key to encrypt firmware file",
        required=True,
        type=hex_to_int,
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Output file, default is <filename>.strn",
        required=False,
        type=output_file_name,
    )
    args = parser.parse_args()

    args.output = args.output if args.output else args.input.split(".")[0] + ".strn"

    with open(args.input, "rb") as f:
        raw_data = f.read()

        raw_blocks = [
            raw_data[i : i + CYPHER_BLOCK_SIZE_BYTES]
            for i in range(0, len(raw_data), CYPHER_BLOCK_SIZE_BYTES)
        ]

        if len(raw_blocks[-1]) < CYPHER_BLOCK_SIZE_BYTES:
            raw_blocks[-1] += b'\xff' * (CYPHER_BLOCK_SIZE_BYTES - len(raw_blocks[-1]))

        cipher = SpeckCipher(args.key, key_size=KEY_SIZE, block_size=CYPHER_BLOCK_SIZE)

        encrypted_blocks = []
        for block in raw_blocks:
            half_block_size = CYPHER_BLOCK_SIZE_BYTES // 2
            upper_word = int.from_bytes(block[:half_block_size], "little")
            lower_word = int.from_bytes(block[half_block_size:], "little")

            encrypted_block = cipher.encrypt_function(upper_word, lower_word)

            half_blocks = []
            half_blocks.append(encrypted_block[0].to_bytes(4, "little"))
            half_blocks.append(encrypted_block[1].to_bytes(4, "little"))
            encrypted_blocks.append(b"".join(half_blocks))

        with open(args.output, "wb") as f:
            f.write(b"".join(encrypted_blocks))


if __name__ == "__main__":
    main()
