#!/usr/bin/env python3

import os
import sys

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    test_node = ExecuteProcess(
        cmd=[os.getenv('TEST_EXECUTABLE')],
        name='graceful_controller_tests',
        output='screen'
    )

    ld = LaunchDescription()
    lts = LaunchTestService()
    lts.add_test_action(ld, test_node)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
