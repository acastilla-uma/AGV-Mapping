#!/usr/bin/env python
from __future__ import print_function

import math
import os
import sys
import threading
import time
from datetime import datetime

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


def utc_now():
    return datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'


def parse_float_list(text, expected):
    if text is None:
        return None
    cleaned = text.strip().strip('[]')
    if not cleaned:
        return None
    parts = [p.strip() for p in cleaned.replace(',', ' ').split() if p.strip()]
    if len(parts) != expected:
        raise ValueError('expected {} values, got {}'.format(expected, len(parts)))
    return [float(p) for p in parts]


def read_simple_yaml(path):
    data = {}
    if not path or not os.path.exists(path):
        return data
    with open(path, 'r') as handle:
        for raw_line in handle:
            line = raw_line.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key in ('xyz', 'rpy', 'rpy_degrees'):
                data[key] = parse_float_list(value, 3)
            else:
                data[key] = value
    return data


def write_simple_yaml(path, parent_frame, child_frame, xyz, rpy, notes=''):
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    rpy_degrees = [math.degrees(v) for v in rpy]
    with open(path, 'w') as handle:
        handle.write('parent_frame: {}\n'.format(parent_frame))
        handle.write('child_frame: {}\n'.format(child_frame))
        handle.write('xyz: [{:.6f}, {:.6f}, {:.6f}]\n'.format(*xyz))
        handle.write('rpy: [{:.9f}, {:.9f}, {:.9f}]\n'.format(*rpy))
        handle.write('rpy_degrees: [{:.6f}, {:.6f}, {:.6f}]\n'.format(*rpy_degrees))
        handle.write('created_utc: "{}"\n'.format(utc_now()))
        handle.write('notes: "{}"\n'.format(str(notes).replace('"', "'")))


class CalibrationState(object):
    def __init__(self, parent_frame, child_frame, xyz, rpy, step, rstep):
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.xyz = list(xyz)
        self.rpy = list(rpy)
        self.default_xyz = list(xyz)
        self.default_rpy = list(rpy)
        self.step = step
        self.rstep = rstep
        self.lock = threading.RLock()
        self.running = True

    def snapshot(self):
        with self.lock:
            return self.parent_frame, self.child_frame, list(self.xyz), list(self.rpy), self.step, self.rstep

    def reset(self):
        with self.lock:
            self.xyz = list(self.default_xyz)
            self.rpy = list(self.default_rpy)

    def show(self):
        parent, child, xyz, rpy, step, rstep = self.snapshot()
        deg = [math.degrees(v) for v in rpy]
        return (
            'parent_frame={} child_frame={}\n'
            'CAMERA_XYZ="{:.6f} {:.6f} {:.6f}"\n'
            'CAMERA_RPY="{:.9f} {:.9f} {:.9f}"\n'
            'RPY_DEG="{:.4f} {:.4f} {:.4f}"\n'
            'step={:.6f}m rstep={:.6f}rad ({:.4f}deg)'
        ).format(parent, child, xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2], deg[0], deg[1], deg[2], step, rstep, math.degrees(rstep))

    def nudge(self, axis, sign):
        with self.lock:
            if axis == 'x':
                self.xyz[0] += sign * self.step
            elif axis == 'y':
                self.xyz[1] += sign * self.step
            elif axis == 'z':
                self.xyz[2] += sign * self.step
            elif axis == 'roll':
                self.rpy[0] += sign * self.rstep
            elif axis == 'pitch':
                self.rpy[1] += sign * self.rstep
            elif axis == 'yaw':
                self.rpy[2] += sign * self.rstep
            else:
                raise ValueError('unknown axis {}'.format(axis))


def publish_loop(state, rate_hz):
    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown() and state.running:
        parent, child, xyz, rpy, _, _ = state.snapshot()
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent.lstrip('/')
        msg.child_frame_id = child.lstrip('/')
        msg.transform.translation.x = xyz[0]
        msg.transform.translation.y = xyz[1]
        msg.transform.translation.z = xyz[2]
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        broadcaster.sendTransform(msg)
        rate.sleep()


def print_help():
    print('''
Comandos calibracion:
  x+ x- y+ y- z+ z-              traslacion
  roll+ roll- pitch+ pitch- yaw+ yaw-   rotacion
  step 0.005                     paso traslacion en metros
  rstep 0.1deg                   paso rotacion en grados, o radianes sin sufijo
  show                           imprimir transformacion actual
  reset                          volver a valores iniciales
  load [ruta]                    cargar YAML
  save [ruta] [notas...]         guardar YAML
  help                           ayuda
  quit                           salir
'''.strip())


def parse_rstep(value):
    text = value.strip().lower()
    if text.endswith('deg'):
        return math.radians(float(text[:-3]))
    return float(text)


def load_into_state(state, path):
    data = read_simple_yaml(path)
    with state.lock:
        state.parent_frame = data.get('parent_frame', state.parent_frame)
        state.child_frame = data.get('child_frame', state.child_frame)
        if data.get('xyz') is not None:
            state.xyz = list(data['xyz'])
        if data.get('rpy') is not None:
            state.rpy = list(data['rpy'])
        elif data.get('rpy_degrees') is not None:
            state.rpy = [math.radians(v) for v in data['rpy_degrees']]


def command_loop(state, calibration_file):
    print_help()
    print('\nValores iniciales:\n{}\n'.format(state.show()))
    while not rospy.is_shutdown() and state.running:
        try:
            line = raw_input('calib> ')
        except EOFError:
            break
        except KeyboardInterrupt:
            break
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        try:
            if cmd in ('quit', 'exit', 'q'):
                state.running = False
                break
            elif cmd in ('help', '?'):
                print_help()
            elif cmd == 'show':
                print(state.show())
            elif cmd == 'reset':
                state.reset()
                print(state.show())
            elif cmd == 'step' and len(parts) >= 2:
                with state.lock:
                    state.step = float(parts[1])
                print(state.show())
            elif cmd == 'rstep' and len(parts) >= 2:
                with state.lock:
                    state.rstep = parse_rstep(parts[1])
                print(state.show())
            elif cmd == 'load':
                path = parts[1] if len(parts) >= 2 else calibration_file
                if not path:
                    print('No hay ruta de calibracion para load')
                    continue
                load_into_state(state, path)
                print('Cargado {}'.format(path))
                print(state.show())
            elif cmd == 'save':
                path = parts[1] if len(parts) >= 2 else calibration_file
                notes = ' '.join(parts[2:]) if len(parts) > 2 else ''
                if not path:
                    print('No hay ruta de calibracion para save')
                    continue
                parent, child, xyz, rpy, _, _ = state.snapshot()
                write_simple_yaml(path, parent, child, xyz, rpy, notes)
                print('Guardado {}'.format(path))
            elif cmd in ('x+', 'x-', 'y+', 'y-', 'z+', 'z-'):
                state.nudge(cmd[0], 1 if cmd.endswith('+') else -1)
                print(state.show())
            elif cmd in ('roll+', 'roll-', 'pitch+', 'pitch-', 'yaw+', 'yaw-'):
                axis = cmd[:-1]
                state.nudge(axis, 1 if cmd.endswith('+') else -1)
                print(state.show())
            else:
                print('Comando no reconocido: {}'.format(line))
                print_help()
        except Exception as exc:
            print('ERROR: {}'.format(exc))


def main():
    rospy.init_node('camera_lidar_calibrator')
    calibration_file = rospy.get_param('~calibration_file', '')
    parent_frame = rospy.get_param('~parent_frame', 'base_link')
    child_frame = rospy.get_param('~child_frame', 'camera_link')
    xyz = parse_float_list(rospy.get_param('~xyz', '0.16 0.0 0.20'), 3)
    rpy = parse_float_list(rospy.get_param('~rpy', '0 0 0'), 3)
    step = float(rospy.get_param('~step', 0.01))
    rstep = math.radians(float(rospy.get_param('~rstep_deg', 0.5)))

    state = CalibrationState(parent_frame, child_frame, xyz, rpy, step, rstep)
    if calibration_file and os.path.exists(calibration_file):
        load_into_state(state, calibration_file)

    thread = threading.Thread(target=publish_loop, args=(state, 10.0))
    thread.daemon = True
    thread.start()
    try:
        command_loop(state, calibration_file)
    finally:
        state.running = False
        time.sleep(0.2)
    return 0


if __name__ == '__main__':
    sys.exit(main())
