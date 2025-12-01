import sys
import struct
from pathlib import Path


def read_uleb(data, offset):
    result = 0
    shift = 0
    pos = offset
    while True:
        byte = data[pos]
        pos += 1
        result |= (byte & 0x7F) << shift
        if (byte & 0x80) == 0:
            break
        shift += 7
    return result, pos


def parse_wasm(path: Path):
    data = path.read_bytes()
    if data[:4] != b"\x00asm":
        raise RuntimeError("Invalid wasm magic")
    if data[4:8] != b"\x01\x00\x00\x00":
        raise RuntimeError("Unsupported wasm version")

    offset = 8
    imported_func_count = 0
    func_type_indices = []
    code_bodies = []
    name_map = {}

    while offset < len(data):
        if offset == len(data):
            break
        section_id = data[offset]
        offset += 1
        payload_len, offset = read_uleb(data, offset)
        payload_start = offset
        payload_end = offset + payload_len
        if section_id == 2:  # import section
            count, offset = read_uleb(data, offset)
            for _ in range(count):
                # module name
                name_len, offset = read_uleb(data, offset)
                offset += name_len
                # field name
                field_len, offset = read_uleb(data, offset)
                offset += field_len
                kind = data[offset]
                offset += 1
                if kind == 0:  # function import
                    type_index, offset = read_uleb(data, offset)
                    imported_func_count += 1
                elif kind == 1:  # table
                    element_type = data[offset]
                    offset += 1
                    flags, offset = read_uleb(data, offset)
                    min_val, offset = read_uleb(data, offset)
                    if flags & 1:
                        max_val, offset = read_uleb(data, offset)
                elif kind == 2:  # memory
                    flags, offset = read_uleb(data, offset)
                    min_val, offset = read_uleb(data, offset)
                    if flags & 1:
                        max_val, offset = read_uleb(data, offset)
                elif kind == 3:  # global
                    offset += 1  # content type
                    offset += 1  # mutability
                else:
                    raise RuntimeError(f"Unknown import kind {kind}")
        elif section_id == 3:  # function section
            func_count, offset = read_uleb(data, offset)
            for _ in range(func_count):
                type_index, offset = read_uleb(data, offset)
                func_type_indices.append(type_index)
        elif section_id == 10:  # code section
            func_body_count, offset = read_uleb(data, offset)
            for _ in range(func_body_count):
                body_size, offset = read_uleb(data, offset)
                body_start = offset
                body_end = body_start + body_size
                # skip local decls
                local_count, offset = read_uleb(data, offset)
                for _ in range(local_count):
                    local_num, offset = read_uleb(data, offset)
                    offset += 1  # local type
                offset = body_end
                code_bodies.append((body_start, body_end))
        elif section_id == 0:  # custom
            name_len, offset = read_uleb(data, offset)
            name = data[offset:offset + name_len].decode('utf-8')
            offset += name_len
            if name == 'name':
                sub_offset = offset
                while sub_offset < payload_end:
                    sub_id = data[sub_offset]
                    sub_offset += 1
                    sub_size, sub_offset = read_uleb(data, sub_offset)
                    sub_end = sub_offset + sub_size
                    if sub_id == 1:  # function names
                        count, sub_offset = read_uleb(data, sub_offset)
                        for _ in range(count):
                            idx, sub_offset = read_uleb(data, sub_offset)
                            str_len, sub_offset = read_uleb(data, sub_offset)
                            name_bytes = data[sub_offset:sub_offset + str_len]
                            sub_offset += str_len
                            try:
                                func_name = name_bytes.decode('utf-8')
                            except UnicodeDecodeError:
                                func_name = name_bytes.decode('utf-8', errors='replace')
                            name_map[idx] = func_name
                    else:
                        sub_offset = sub_end
            else:
                offset = payload_end
        else:
            offset = payload_end

    return imported_func_count, code_bodies, name_map


def main():
    if len(sys.argv) < 2:
        print("Usage: analyze_wasm.py <wasm> [offset_hex]")
        return
    wasm_path = Path(sys.argv[1])
    imported_count, bodies, names = parse_wasm(wasm_path)
    offset_hex = None
    if len(sys.argv) >= 3:
        offset_hex = int(sys.argv[2], 16)
    for i, (start, end) in enumerate(bodies):
        func_index = imported_count + i
        name = names.get(func_index, '')
        if offset_hex is not None:
            if start <= offset_hex < end:
                print(f"match func_index={func_index} start=0x{start:x} end=0x{end:x} name={name}")
                break
        else:
            print(f"func_index={func_index} start=0x{start:x} end=0x{end:x} size={end-start} name={name}")
    else:
        if offset_hex is not None:
            print("no match for offset")


if __name__ == '__main__':
    main()
