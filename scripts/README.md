# SD Card Tools: Extract and Clear

Two Python scripts for working with SD cards written raw by the logging firmware (`logging_formatter.c`):

| Script | What it does |
|---|---|
| `sd_extract.py` | Reads the card, checks each 32 KiB chunk's header and CRC, and rebuilds the logged files. Read-only. |
| `sd_erase.py` | Overwrites every chunk that contains data with zeros, so the card is ready for a new flight. **Destroys data.** |

Both need only Python 3 (no extra packages) and work on macOS, Windows and Linux. They open the card as a raw device, so they must run with admin rights: `sudo` on macOS and Linux, an Administrator terminal on Windows.

Jump to: [macOS](#before-you-start-macos) · [Windows](#windows) · [Linux](#linux)

The examples below use macOS device names. On Windows, use `\\.\PhysicalDriveN` instead of `/dev/rdiskN` and drop `sudo` (see [Windows](#windows)).

---

## Before you start (macOS)

### 1. Insert the card

macOS will say **"The disk you attached was not readable by this computer."** This is expected: the card has no filesystem. Click **Ignore**. Do **not** click Initialize (that formats the card).

### 2. Find the card's disk number

```
diskutil list
```

Look for a disk whose size matches the card, e.g. `/dev/disk4`. A card in the Mac's built-in SD slot shows as `internal, physical`, the same as the Mac's own drive, so always confirm before running anything:

```
diskutil info /dev/disk4 | grep -E "Media Name|Protocol|Disk Size|Removable"
```

| It's the SD card if | It's the Mac's drive (do not touch) if |
|---|---|
| Protocol: `Secure Digital` | Protocol: `Apple Fabric`, `PCI-Express` or `NVMe` |
| Media name like `SD Card Reader` | Media name like `APPLE SSD` |
| Removable Media: `Removable` | Removable Media: `Fixed` |
| Size matches the card | Usually `disk0`, with `Apple_APFS` partitions |

Disk numbers can change every time the card is inserted. Check again each time.

### 3. Unmount it

```
diskutil unmountDisk /dev/disk4
```

Then run the script on the **raw** device, `/dev/rdisk4` (note the `r`). Unmounting uses `disk4`; the scripts use `rdisk4`.

---

## Extracting logs: `sd_extract.py`

```
sudo python3 sd_extract.py /dev/rdisk4 -o sd_files
```

The script never writes to the card. It saves the rebuilt files in the folder given by `-o` (default `sd_files`).

### Chunk format it expects

The card is read in 32768-byte chunks. Chunk 0 is reserved by the firmware and skipped. All multi-byte fields are big-endian.

| Bytes | Contents |
|---|---|
| 0–3 | Constant header `43 6B CF FF` |
| 4–5 | Top 13 bits: file identifier (one per boot). Low 3 bits: data type (0 DATA, 1 ERROR, 2 TEXT) |
| 6–9 | 32-bit index of this chunk within its file (0, 1, 2, …) |
| 10–32765 | Payload (32756 bytes) |
| 32766–32767 | CRC-16/MODBUS over bytes 0–32765, high byte first |

The CRC is CRC-16/MODBUS: polynomial `0xA001` (reflected), start value `0xFFFF`, no final XOR. It is the same CRC the firmware computes.

### Output

Chunks with the same file identifier and data type are joined in chunk-index order into one file:

```
file_<identifier>_<TYPE>.bin     (DATA and ERROR)
file_<identifier>_<TYPE>.txt     (TEXT, with trailing zero padding removed)
```

At the end the script prints a summary:

```
Read 40 chunks (from chunk 1): 8 good, 0 bad CRC, 0 wrong header, 32 blank

Writing 1 file(s) to sd_files/
  file_0000_TEXT.txt               8 chunk(s)      12345 bytes
```

A file is flagged with `<-- missing index [...]` if some of its chunks were not found or failed the CRC.

### Options

| Option | Default | Meaning |
|---|---|---|
| `-o DIR`, `--out DIR` | `sd_files` | Output folder |
| `--keep-bad` | off | Also include chunks that fail the CRC |
| `--stop-after-blank N` | `32` | Stop after N blank chunks in a row (1 MiB). `0` reads the whole card |
| `--crc-start N` | `0` | First byte the CRC covers. Use `10` for cards written by older firmware that CRC'd the payload only |
| `--crc-zero-pad N` | `0` | Old firmware only: the CRC covered N bytes past byte 32765 that the footer then overwrote |
| `--crc-order big\|little` | `big` | Byte order of the stored CRC |

If **every** CRC fails, the script tests the likely CRC settings and prints the options that match, for example:

```
match: --crc-start 10 --crc-zero-pad 2 --crc-order big
Re-run with those options.
```

If nothing matches, the bytes on the card are not the bytes the firmware computed the CRC over.

### Reading the results

| Message | Meaning |
|---|---|
| `CRC mismatch ... skipped` | The chunk's data doesn't match its CRC: corrupted, or written by firmware with a different CRC layout |
| `header ... skipped` | The chunk isn't in the logger's format |
| `header eb 58 90 ... (FAT/exFAT boot sector ...)` | Leftover filesystem from when the card was formatted. Erase the card |
| A file with 0 bytes | Its chunks passed the CRC but their payload was all zeros |

---

## Clearing the card: `sd_erase.py`

> **Warning:** this permanently destroys everything on the device you give it. Confirm the disk number first (see above).

Check first with a dry run, which writes nothing:

```
sudo python3 sd_erase.py /dev/rdisk4 --dry-run
```

Then erase:

```
sudo python3 sd_erase.py /dev/rdisk4
```

It asks you to type the device path again to confirm. It scans the card in 1 MiB pieces and overwrites with zeros only the pieces that contain data; pieces that are already all `0x00` or all `0xFF` are skipped. It works on the raw device, so it also removes any partition table or filesystem.

### Options

| Option | Meaning |
|---|---|
| `--dry-run` | Report how much would be erased, without writing |
| `--yes` | Skip the confirmation prompt |
| `--stop-after-blank MB` | Stop after MB blank MiB in a row instead of scanning the whole card. Faster, but only safe if all data is at the start of the card |

Erase the whole card (no `--stop-after-blank`) the first time a card is used for logging. Leftover data anywhere on the card can confuse the firmware's search for the first free chunk.

The last line says `erased`, or `would erase` for a dry run. To confirm the card is blank, run `sd_extract.py` on it: it should report 0 good chunks and no files.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Resource busy` | The card is still mounted. Run `diskutil unmountDisk /dev/diskN` (or `diskutil unmountDisk force /dev/diskN`) and run the script straight after. Close any Finder window showing the card |
| `Permission denied` | Run with `sudo` |
| `can't open file '.../sd_extract.py'` | Give the script's full path, e.g. `~/Downloads/sd_extract.py`, or drag the file into Terminal to paste its path |
| Erase reports `scanned 0.00 GiB` | The device couldn't be read: wrong path, or the card isn't inserted |
| Card won't erase | Check the lock switch on the side of the SD card or microSD adapter |
| Want to use the card as normal storage again | In Disk Utility, erase it as MS-DOS (FAT) for 32 GB and under, or ExFAT for larger cards |

## Windows

### 1. Insert the card

Windows may say **"You need to format the disk in drive X: before you can use it."** Click **Cancel**. Formatting would write a new filesystem over the logs.

### 2. Open an Administrator terminal

Right-click **PowerShell** (or **Terminal**) in the Start menu and choose **Run as administrator**. There is no `sudo` on Windows; the scripts need this instead. Run them with `python` (or `py` if `python` isn't found).

### 3. Find the card's disk number

```
Get-Disk
```

Look for the disk whose size matches the card. The **Bus Type** is `SD` for a built-in card slot or `USB` for a USB card reader. The PC's own drive is usually disk 0 with Bus Type `NVMe`, `SATA` or `RAID`, and is marked as the boot disk. **Never use the boot disk.**

If disk 2 is the card, its raw device path is `\\.\PhysicalDrive2`. Disk numbers can change when the card is re-inserted, so check each time.

### 4. Extract logs

Reading works without any extra steps:

```
python $HOME\Downloads\sd_extract.py \\.\PhysicalDrive2 -o sd_files
```

### 5. Erase the card

Windows does not let programs write to the part of a disk that holds a mounted volume (a drive letter). A card that has never been formatted, or that has only logger data on it, has no volume, so you can run the erase directly:

```
python $HOME\Downloads\sd_erase.py \\.\PhysicalDrive2 --dry-run
python $HOME\Downloads\sd_erase.py \\.\PhysicalDrive2
```

If the card has a drive letter, or the erase fails with `Permission denied` or `Access is denied`, remove its volume first with `diskpart`:

```
diskpart
list disk
select disk 2
clean
exit
```

Check `select disk` uses the card's number from `list disk`: `clean` wipes the partition table of whichever disk is selected. Then run `sd_erase.py` again.

`clean all` in diskpart zeroes the entire card and is an alternative to `sd_erase.py`, but it writes every byte, so it is much slower on a large card.

### Windows troubleshooting

| Problem | Fix |
|---|---|
| `Permission denied` when opening the card | The terminal isn't running as Administrator |
| `Access is denied` or `Permission denied` during the erase | The card has a mounted volume; run `diskpart` → `select disk N` → `clean` (above) |
| `python` is not recognized | Use `py`, or install Python from python.org and tick **Add Python to PATH** |
| `can't open file ...sd_extract.py` | Give the full path, e.g. `$HOME\Downloads\sd_extract.py` |
| Formatting prompt every time the card is inserted | Expected while the card has no filesystem; click **Cancel** |

---

## Linux

The same commands work with the Linux device name, e.g. `/dev/sdb`. Find it with `lsblk`, and unmount any mounted partitions with `sudo umount /dev/sdb1` first.
