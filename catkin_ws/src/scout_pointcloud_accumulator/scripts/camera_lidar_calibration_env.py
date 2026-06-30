#!/usr/bin/env python
from __future__ import print_function

import math
import sys

try:
    from pipes import quote as shell_quote
except ImportError:
    from shlex import quote as shell_quote


def parse_list(value):
    value = value.strip().strip('[]')
    return [float(v.strip()) for v in value.replace(',', ' ').split() if v.strip()]


def read_simple_yaml(path):
    data = {}
    with open(path, 'r') as handle:
        for raw in handle:
            line = raw.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            data[key.strip()] = value.strip().strip('"').strip("'")
    return data


def emit(name, value):
    print('{}={}'.format(name, shell_quote(str(value))))


def main(argv):
    if len(argv) != 2:
        print('Usage: camera_lidar_calibration_env.py calibration.yaml', file=sys.stderr)
        return 2
    data = read_simple_yaml(argv[1])
    if 'parent_frame' in data:
        emit('YAML_CAMERA_PARENT_FRAME', data['parent_frame'])
    if 'child_frame' in data:
        emit('YAML_CAMERA_CHILD_FRAME', data['child_frame'])
    if 'xyz' in data:
        xyz = parse_list(data['xyz'])
        if len(xyz) == 3:
            emit('YAML_CAMERA_XYZ', '{:.6f} {:.6f} {:.6f}'.format(*xyz))
    if 'rpy' in data:
        rpy = parse_list(data['rpy'])
        if len(rpy) == 3:
            emit('YAML_CAMERA_RPY', '{:.9f} {:.9f} {:.9f}'.format(*rpy))
    elif 'rpy_degrees' in data:
        deg = parse_list(data['rpy_degrees'])
        if len(deg) == 3:
            rpy = [math.radians(v) for v in deg]
            emit('YAML_CAMERA_RPY', '{:.9f} {:.9f} {:.9f}'.format(*rpy))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
