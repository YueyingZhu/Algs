#!/usr/bin/env python3
import sys
import csv

def convert_csv_to_tum(csv_file, tum_file):
    """Convert CSV ground truth to TUM format"""
    with open(csv_file, 'r') as csvfile:
        with open(tum_file, 'w') as tumfile:
            for line in csvfile:
                parts = line.strip().split()
                if len(parts) >= 8:
                    # CSV format: timestamp x y z qx qy qz qw
                    # TUM format: timestamp x y z qx qy qz qw (same format)
                    tumfile.write(f"{parts[0]} {parts[1]} {parts[2]} {parts[3]} {parts[4]} {parts[5]} {parts[6]} {parts[7]}\n")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 convert_gt.py <input.csv> <output.txt>")
        sys.exit(1)
    
    convert_csv_to_tum(sys.argv[1], sys.argv[2])
    print(f"Converted {sys.argv[1]} to {sys.argv[2]}")
