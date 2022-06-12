#!/usr/bin/env python3

import os
import sys

from launch import LaunchDescription
from launch import LaunchService
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    config_file = os.path.join(os.getenv('CONFIG_DIR'), 'config.yaml')

    test_node = Node(
        executable=[os.getenv('TEST_EXECUTABLE')],
        name='graceful_controller_tests',
        parameters=[config_file],
        output='screen',
    )

    ld = LaunchDescription()
    lts = LaunchTestService()
    lts.add_test_action(ld, test_node)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
