#!/usr/bin/env python3

import os.path as osp
import rospy

from movedrone_setup_assistant import KeyboardEventChecker


if __name__ == '__main__':
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)
    node = KeyboardEventChecker()
    node.run()
