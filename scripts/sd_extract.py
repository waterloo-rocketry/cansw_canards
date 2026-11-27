#!/usr/bin/env python3
"""Extract logged files from a raw SD card written by logging_formatter.c.
Read-only: never writes to the card.

Chunk layout (CHUNK_SIZE = 32768 bytes, multi-byte fields big-endian),
matching logging_formatter_write():

    bytes 0-3        constant header 0x43 0x6B 0xCF 0xFF
    bytes 4-5        bits 15..3 = 13-bit file identifier (one per boot)
                     bits  2..0 = data type (0 DATA, 1 ERROR, 2 TEXT)
    bytes 6-9        32-bit chunk index within the file (0, 1, 2, ...)
    bytes 10-32765   payload (32756 bytes)
    bytes 32766-7    CRC-16/MODBUS of bytes 0-32765 (header + payload),
                     high byte first

Chunk 0 is reserved by the firmware and is skipped.
Chunks with the same (identifier, type) are joined in chunk-index order into
file_<identifier>_<TYPE>.bin (.txt for TEXT).

Usage:
    sudo python3 sd_extract.py /dev/rdiskN -o sd_files
Options:
    --crc-start N        first byte the CRC covers (default 0 = header + payload;
                         10 for old firmware that CRC'd the payload only)
    --crc-zero-pad N     the firmware CRC'd N bytes past byte 32765 that the
                         footer then overwrote; assume they were zero
    --crc-order big|little   byte order of the stored CRC (default big)
    --keep-bad           also write chunks that fail the CRC
    --stop-after-blank N stop after N consecutive blank chunks (default 32,
                         0 = read the whole card)
"""
import argparse
import os
import sys

CHUNK_SIZE = 32768
HEADER = bytes([0x43, 0x6B, 0xCF, 0xFF])
PAYLOAD_START = 10
CRC_SIZE = 2
PAYLOAD_END = CHUNK_SIZE - CRC_SIZE          # 32766
PAYLOAD_SIZE = PAYLOAD_END - PAYLOAD_START   # 32756
FIRST_CHUNK = 1                              # chunk 0 is reserved
TYPE_NAMES = {0: "DATA", 1: "ERROR", 2: "TEXT"}


def _make_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        table.append(crc)
    return table


CRC_TABLE = _make_table()


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def is_blank(chunk: bytes) -> bool:
    return chunk.count(0) == len(chunk) or chunk.count(0xFF) == len(chunk)


def crc_continue(crc: int, data: bytes) -> int:
    for b in data:
        crc = (crc >> 8) ^ CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def diagnose(chunks):
    """Try the plausible CRC coverages; return the ones that match every chunk."""
    results = None
    for chunk in chunks:
        stored = {o: int.from_bytes(chunk[PAYLOAD_END:], o) for o in ("big", "little")}
        found = set()
        for start in (0, 4, 6, 10):
            crc = crc16_modbus(chunk[start:PAYLOAD_END])
            for pad in range(0, 9):
                if pad:
                    crc = crc_continue(crc, b"\0")
                for order, value in stored.items():
                    if crc == value:
                        found.add((start, pad, order))
        results = found if results is None else results & found
    return results or set()


def parse_chunk(chunk: bytes, crc_start: int, crc_pad: int, crc_order: str):
    word = int.from_bytes(chunk[4:6], "big")
    covered = chunk[crc_start:PAYLOAD_END] + bytes(crc_pad)
    stored = int.from_bytes(chunk[PAYLOAD_END:], crc_order)
    computed = crc16_modbus(covered)
    return {
        "header_ok": chunk[:4] == HEADER,
        "header": chunk[:4],
        "file_id": word >> 3,
        "type": word & 0b111,
        "index": int.from_bytes(chunk[6:10], "big"),
        "payload": chunk[PAYLOAD_START:PAYLOAD_END],
        "crc_ok": stored == computed,
        "crc_stored": stored,
        "crc_computed": computed,
    }


def main() -> int:
    p = argparse.ArgumentParser(description="Extract logged files from a raw SD card.")
    p.add_argument("device", help="raw device (/dev/rdiskN, /dev/sdX) or a card image file")
    p.add_argument("-o", "--out", default="sd_files", help="output folder (default: sd_files)")
    p.add_argument("--crc-start", type=int, default=0,
                   help="first byte covered by the CRC (default 0; 10 for old firmware)")
    p.add_argument("--crc-zero-pad", type=int, default=0,
                   help="zero bytes the firmware CRC'd after byte 32765 (default 0)")
    p.add_argument("--crc-order", choices=["big", "little"], default="big",
                   help="byte order of the stored CRC (default big)")
    p.add_argument("--keep-bad", action="store_true", help="include chunks that fail the CRC")
    p.add_argument("--stop-after-blank", type=int, default=32, metavar="N",
                   help="stop after N consecutive blank chunks (0 = read whole card)")
    args = p.parse_args()

    try:
        dev = open(args.device, "rb", buffering=0)
    except PermissionError:
        print("Permission denied: run with sudo.")
        return 1
    except OSError as e:
        print(f"Could not open {args.device}: {e}")
        print("If it says 'Resource busy', run: diskutil unmountDisk /dev/diskN")
        return 1

    files = {}  # (file_id, type) -> {index: payload}
    stats = {"read": 0, "blank": 0, "bad_header": 0, "bad_crc": 0, "good": 0}
    blank_run = 0
    chunk_no = FIRST_CHUNK
    bad_samples = []

    with dev:
        dev.seek(FIRST_CHUNK * CHUNK_SIZE)
        while True:
            try:
                chunk = dev.read(CHUNK_SIZE)
            except OSError as e:
                print(f"Read stopped at chunk {chunk_no}: {e}")
                break
            if len(chunk) < CHUNK_SIZE:
                break
            stats["read"] += 1
            this_chunk = chunk_no
            chunk_no += 1

            if is_blank(chunk):
                stats["blank"] += 1
                blank_run += 1
                if args.stop_after_blank and blank_run >= args.stop_after_blank:
                    break
                continue
            blank_run = 0

            c = parse_chunk(chunk, args.crc_start, args.crc_zero_pad, args.crc_order)
            if not c["header_ok"]:
                stats["bad_header"] += 1
                note = ""
                if chunk[:3] in (b"\xeb\x58\x90", b"\xeb\x3c\x90", b"\xeb\x76\x90"):
                    note = " (FAT/exFAT boot sector: card was never erased)"
                print(f"  chunk {this_chunk}: header {c['header'].hex(' ')}, skipped{note}")
                continue

            tname = TYPE_NAMES.get(c["type"], f"TYPE{c['type']}")
            if not c["crc_ok"]:
                stats["bad_crc"] += 1
                if len(bad_samples) < 5:
                    bad_samples.append(chunk)
                print(f"  chunk {this_chunk}: CRC mismatch (stored 0x{c['crc_stored']:04X}, "
                      f"computed 0x{c['crc_computed']:04X}), file {c['file_id']} {tname} "
                      f"index {c['index']}" + ("" if args.keep_bad else ", skipped"))
                if not args.keep_bad:
                    continue
            else:
                stats["good"] += 1

            entry = files.setdefault((c["file_id"], c["type"]), {})
            if c["index"] in entry:
                print(f"  chunk {this_chunk}: duplicate index {c['index']} for file "
                      f"{c['file_id']} {tname}, keeping the later one")
            entry[c["index"]] = c["payload"]

    print(f"\nRead {stats['read']} chunks (from chunk {FIRST_CHUNK}): {stats['good']} good, "
          f"{stats['bad_crc']} bad CRC, {stats['bad_header']} wrong header, {stats['blank']} blank")
    if bad_samples and stats["good"] == 0:
        print("\nEvery CRC failed. Testing which bytes the firmware's CRC covers...")
        matches = diagnose(bad_samples)
        if matches:
            for start, pad, order in sorted(matches):
                print(f"  match: --crc-start {start} --crc-zero-pad {pad} --crc-order {order}")
                if pad:
                    print(f"    (the firmware CRC'd {32766 - start + pad} bytes, so the payload ran "
                          f"{pad} byte(s) into the footer and the CRC overwrote them)")
            print("Re-run with those options.")
        else:
            print("  No coverage matched. The bytes on the card are probably not the bytes the")
            print("  CPU computed the CRC over (e.g. D-cache not cleaned before the SD DMA write).")

    if not files:
        print("No files found.")
        return 0

    os.makedirs(args.out, exist_ok=True)
    print(f"\nWriting {len(files)} file(s) to {args.out}/")
    for (file_id, dtype), chunks in sorted(files.items()):
        tname = TYPE_NAMES.get(dtype, f"TYPE{dtype}")
        ext = "txt" if dtype == 2 else "bin"
        name = f"file_{file_id:04d}_{tname}.{ext}"
        indices = sorted(chunks)
        data = b"".join(chunks[i] for i in indices)
        if dtype == 2:
            data = data.rstrip(b"\x00")
        with open(os.path.join(args.out, name), "wb") as f:
            f.write(data)
        missing = sorted(set(range(indices[-1] + 1)) - set(indices))
        note = f"   <-- missing index {missing[:10]}{'...' if len(missing) > 10 else ''}" if missing else ""
        print(f"  {name:<28} {len(indices):>5} chunk(s) {len(data):>10} bytes{note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
