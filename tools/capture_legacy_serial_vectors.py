#!/usr/bin/env python3
"""Execute ONLY the inspected original TX method against an in-memory fake port.

Requires original libscrc==1.8.1 for fixture generation, not runtime. Does not
import the original ROS node, run its initializer, or open any device.
"""
import argparse
import ast
import json
from pathlib import Path
import struct
from types import SimpleNamespace
import libscrc


def capture(source):
    tree = ast.parse(source.read_text(encoding='utf-8'))
    classes = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SerialNode']
    methods = [n for n in classes[-1].body if isinstance(n, ast.FunctionDef) and n.name == 'periodic_send_to_stm32']
    if len(methods) != 1:
        raise ValueError('Expected the original inspected SerialNode TX method')
    scope = {'struct': struct, 'libscrc': libscrc}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), scope)
    cases = [(0., 0., 0., 3, 0, 0), (.5, -.25, 30., 3, 5, 1),
             (-.2, .3, -179.5, 1, 0, 0), (.123456789, -.234567891, 90., 2, 255, 1),
             (0., -.5, 180., 3, 7, 0), (-0., 0., -0., 3, 0, 0),
             (.3, .4, 359.75, 255, 255, 1), (-.5, .5, -360., 2, 4, 1)]
    result = []
    for x, y, yaw, stance, region, aligned in cases:
        writes = []
        def fail():
            raise RuntimeError('Original TX method failed')
        node = SimpleNamespace(serial_conn=SimpleNamespace(is_open=True, write=writes.append),
            latest_x=x, latest_y=y, yaw_angle=yaw, stance_running_state=stance,
            region_code=region, big_yaw_aligned=aligned, tx_reserved_fields=[1.]*8, get_logger=fail)
        scope['periodic_send_to_stm32'](node)
        if len(writes) != 1 or len(writes[0]) != 54:
            raise ValueError('Original TX method did not produce one 54-byte packet')
        result.append({'input': [x, y, yaw, stance, region, aligned], 'packet_hex': writes[0].hex()})
    return {'source': 'frozen original my_serial_py/serialpy_node.py::SerialNode.periodic_send_to_stm32',
            'crc_library': 'libscrc==1.8.1', 'physical_io': False, 'cases': result}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=Path(__file__).parents[1]/'src/my_serial_py/my_serial_py/serialpy_node.py')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(capture(args.source), indent=2)+'\n', encoding='utf-8')
