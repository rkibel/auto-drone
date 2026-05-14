import struct
import zlib


def write_png(path, width, height, rgb_bytes):
    """Write an 8-bit RGB PNG using only the Python standard library."""
    if len(rgb_bytes) != width * height * 3:
        raise ValueError("rgb_bytes length does not match width * height * 3")

    def chunk(kind, data):
        payload = kind + data
        checksum = zlib.crc32(payload) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + payload + struct.pack(">I", checksum)

    rows = []
    stride = width * 3
    for y in range(height):
        start = y * stride
        rows.append(b"\x00" + rgb_bytes[start : start + stride])

    png = [
        b"\x89PNG\r\n\x1a\n",
        chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)),
        chunk(b"IDAT", zlib.compress(b"".join(rows), level=6)),
        chunk(b"IEND", b""),
    ]

    with open(path, "wb") as file:
        file.write(b"".join(png))
