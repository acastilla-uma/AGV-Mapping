#!/usr/bin/env python
from __future__ import print_function

import argparse
import math
import sys
import time

import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class OrientationStats(object):
    def __init__(self, name):
        self.name = name
        self.samples = []

    def add(self, stamp, quat):
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.samples.append((stamp, roll, pitch, yaw))

    def report(self, warn_deg):
        if not self.samples:
            print('ERROR: no samples for {}'.format(self.name))
            return False

        rolls = [deg(s[1]) for s in self.samples]
        pitches = [deg(s[2]) for s in self.samples]
        yaws = [deg(s[3]) for s in self.samples]
        max_abs_roll = max(abs(v) for v in rolls)
        max_abs_pitch = max(abs(v) for v in pitches)
        ok = max_abs_roll <= warn_deg and max_abs_pitch <= warn_deg

        print('')
        print(self.name)
        print('  samples: {}'.format(len(self.samples)))
        print('  roll_deg:  min={:.4f} max={:.4f} max_abs={:.4f}'.format(min(rolls), max(rolls), max_abs_roll))
        print('  pitch_deg: min={:.4f} max={:.4f} max_abs={:.4f}'.format(min(pitches), max(pitches), max_abs_pitch))
        print('  yaw_deg:   min={:.4f} max={:.4f}'.format(min(yaws), max(yaws)))
        if not ok:
            print('  WARNING: roll/pitch exceeds {:.2f} deg'.format(warn_deg))
        return ok


def deg(value):
    return value * 180.0 / math.pi


def parse_tf_pair(raw):
    if ':' not in raw:
        raise argparse.ArgumentTypeError('TF pair must be target:source, got {}'.format(raw))
    target, source = raw.split(':', 1)
    target = target.strip()
    source = source.strip()
    if not target or not source:
        raise argparse.ArgumentTypeError('TF pair must be target:source, got {}'.format(raw))
    return target, source


def main():
    parser = argparse.ArgumentParser(
        description='Measure roll/pitch drift in LeGO-LOAM odometry and the TF chains used by the accumulator.')
    parser.add_argument('--duration', type=float, default=30.0, help='Seconds to sample.')
    parser.add_argument('--warn-deg', type=float, default=1.0, help='Warn if max abs roll/pitch exceeds this value.')
    parser.add_argument('--odom-topic', action='append', default=[],
                        help='Odometry topic to sample. Can be repeated.')
    parser.add_argument('--tf-pair', action='append', default=[], type=parse_tf_pair,
                        help='TF pair target:source to sample, e.g. map:base_link. Can be repeated.')
    parser.add_argument('--allow-missing', action='store_true',
                        help='Do not fail if a requested topic/frame has no samples.')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if not args.odom_topic:
        args.odom_topic = ['/aft_mapped_to_init', '/integrated_to_init']
    if not args.tf_pair:
        args.tf_pair = [('map', 'base_link'), ('map', 'camera_link')]

    rospy.init_node('check_lego_roll_pitch', anonymous=True)

    odom_stats = []
    subscribers = []
    for topic in args.odom_topic:
        stats = OrientationStats('odom {}'.format(topic))
        odom_stats.append(stats)

        def make_callback(stats_obj):
            def callback(msg):
                q = msg.pose.pose.orientation
                stats_obj.add(msg.header.stamp.to_sec(), [q.x, q.y, q.z, q.w])
            return callback

        subscribers.append(rospy.Subscriber(topic, Odometry, make_callback(stats), queue_size=50))

    tf_listener = tf.TransformListener()
    tf_stats = [OrientationStats('tf {} -> {}'.format(target, source)) for target, source in args.tf_pair]

    deadline = time.time() + args.duration
    print('Sampling for {:.1f}s...'.format(args.duration))
    print('Odometry topics: {}'.format(', '.join(args.odom_topic)))
    print('TF pairs: {}'.format(', '.join(['{}:{}'.format(t, s) for t, s in args.tf_pair])))

    while time.time() < deadline and not rospy.is_shutdown():
        for idx, (target, source) in enumerate(args.tf_pair):
            try:
                _, quat = tf_listener.lookupTransform(target, source, rospy.Time(0))
                tf_stats[idx].add(rospy.Time.now().to_sec(), quat)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        rospy.sleep(0.1)

    all_ok = True
    for stats in odom_stats + tf_stats:
        ok = stats.report(args.warn_deg)
        if not ok and (stats.samples or not args.allow_missing):
            all_ok = False

    if all_ok:
        print('')
        print('OK: all sampled roll/pitch values stayed within {:.2f} deg'.format(args.warn_deg))
        return 0

    print('')
    print('WARNING: one or more sampled odometry/TF chains exceeded the limit or produced no samples.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
