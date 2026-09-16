#!/usr/bin/env python3
"""Diagnostic: report the first Gazebo links containing non-finite state."""

import math
import time
import rospy
from gazebo_msgs.msg import LinkStates


def values(pose, twist):
    return (pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z)


def main():
    rospy.init_node('bumblebee_probe_links', anonymous=True, disable_signals=True)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        try:
            msg = rospy.wait_for_message('/gazebo/link_states', LinkStates, timeout=2)
        except rospy.ROSException:
            continue
        bad = [name for name, pose, twist in zip(msg.name, msg.pose, msg.twist) if not all(math.isfinite(value) for value in values(pose, twist))]
        if bad:
            print('\n'.join(bad))
            return
    raise SystemExit('No valid connection detected within 30 seconds')


if __name__ == '__main__':
    main()
