#!/usr/bin/env python3
"""Merge bootloader and application HEX files into a combined firmware image."""

import sys
from intelhex import IntelHex


def merge_hex(bootloader_hex_file, application_hex_file, combined_hex_file):
    """
    Merge bootloader.hex and application.hex into combined.hex.
    
    Args:
        bootloader_hex_file: Path to bootloader HEX file
        application_hex_file: Path to application HEX file
        combined_hex_file: Output path for combined HEX file
    """
    print(f"Loading bootloader: {bootloader_hex_file}")
    ih_boot = IntelHex(bootloader_hex_file)
    
    print(f"Loading application: {application_hex_file}")
    ih_app = IntelHex(application_hex_file)
    
    # Clear application start address (IntelHex quirk)
    if ih_app.start_addr is not None:
        print(f"Ignoring application start address: {ih_app.start_addr}")
        ih_app.start_addr = None
    
    # Merge with overlap detection
    print("Merging HEX files...")
    try:
        ih_boot.merge(ih_app, overlap='error')
    except ValueError as e:
        print(f"ERROR: Overlap detected during merge: {e}")
        sys.exit(1)
    
    # Write combined output
    print(f"Writing combined image: {combined_hex_file}")
    ih_boot.write_hex_file(combined_hex_file)
    print(f"SUCCESS: Combined firmware written to {combined_hex_file}")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <bootloader.hex> <application.hex> <combined.hex>")
        sys.exit(1)
    
    merge_hex(sys.argv[1], sys.argv[2], sys.argv[3])
