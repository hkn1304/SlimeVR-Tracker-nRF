#!/usr/bin/env python3
"""Convert BHI385 binary firmware to C source with const uint8_t array."""
import sys

def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <input.fw> <output.c> <varname>")
        sys.exit(1)
    
    in_path, out_path, varname = sys.argv[1], sys.argv[2], sys.argv[3]
    
    with open(in_path, 'rb') as f:
        data = f.read()
    
    with open(out_path, 'w') as f:
        f.write('#include <stdint.h>\n\n')
        f.write(f'const uint8_t {varname}[] = {{\n')
        for i in range(0, len(data), 16):
            chunk = data[i:i+16]
            f.write('    ' + ', '.join(f'0x{b:02x}' for b in chunk) + ',\n')
        f.write('};\n\n')
        f.write(f'const uint32_t {varname}_size = sizeof({varname});\n')

if __name__ == '__main__':
    main()
