#!/usr/bin/env python3

import sys

for line in sys.stdin:
    bitStrings = line.strip().split(",")
    if len(bitStrings) == 8:
        bits = [ {"0x00":0,"0xFF":1}[bitString] for bitString in bitStrings[0:7]]
        byte = bits[0] * 1 + bits[1] * 2 + bits[2] * 4 + bits[3] * 8 + bits[4] * 16 + bits[5] * 32 + bits[6] * 64;
        print("0x%02X, " % byte, end='')
    else:
        print("\n" + line, end='')
