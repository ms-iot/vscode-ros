# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

import asyncio
import argparse
import os
import sys
import subprocess
from io import StringIO
from typing import cast
from typing import Dict
from typing import List
from typing import Text
from typing import Tuple
from typing import Union

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
from ros2launch.api import get_share_file_path_from_package
from ros2launch.api import is_launch_file
from ros2launch.api import launch_a_launch_file
from ros2launch.api import LaunchFileNameCompleter
from ros2launch.api import MultipleLaunchFilesError
from ros2launch.api import print_a_launch_file
from ros2launch.api import print_arguments_of_launch_file
import launch
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.utilities import is_a
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch import Action
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch_ros.actions import PushRosNamespace
from launch_ros.ros_adapters import get_ros_adapter


def parse_launch_arguments(launch_arguments: List[Text]) -> List[Tuple[Text, Text]]:
    """Parse the given launch arguments from the command line, into list of tuples for launch."""
    parsed_launch_arguments = {}  # type: ignore; 3.7+ dict is ordered.
    for argument in launch_arguments:
        count = argument.count(':=')
        if count == 0 or argument.startswith(':=') or (count == 1 and argument.endswith(':=')):
            raise RuntimeError(
                "malformed launch argument '{}', expected format '<name>:=<value>'"
                .format(argument))
        name, value = argument.split(':=', maxsplit=1)
        parsed_launch_arguments[name] = value  # last one wins is intentional
    return parsed_launch_arguments.items()


def find_files(file_name):
    search_results = [file_name]
    if os.name == 'nt':
        output = subprocess.Popen(['where', file_name], stdout=subprocess.PIPE, shell=True).communicate()[0]
        output = output.decode()
        search_results = output.split(os.linesep)
    else:
        # Do we need this on Linux?
        pass
    return search_results[0]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    arg = parser.add_argument(
        'launch_full_path',
        help='Full file path to the launch file')
    arg = parser.add_argument(
        'launch_arguments',
        nargs='*',
        help="Arguments to the launch file; '<name>:=<value>' (for duplicates, last one wins)")

    args = parser.parse_args()

    path = None
    launch_arguments = []

    if os.path.exists(args.launch_full_path):
        path = args.launch_full_path
    else:
        raise RuntimeError('No launch file supplied')

    launch_arguments.extend(args.launch_arguments)
    parsed_launch_arguments = parse_launch_arguments(launch_arguments)

    launch_description = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.AnyLaunchDescriptionSource(
                path
            ),
            launch_arguments=parsed_launch_arguments,
        ),
    ])
    walker = []
    walker.append(launch_description)
    ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
    context = LaunchContext(argv=launch_arguments)
    context._set_asyncio_loop(asyncio.get_event_loop())

    try:
        # * Here we mimic the run loop inside launch_service,
        #   but without actually kicking off the processes.
        # * Traverse the sub entities by DFS.
        # * Shadow the stdout to avoid random print outputs.
        mystring = StringIO()
        sys.stdout = mystring
        while walker:
            entity = walker.pop()
            visit_future = entity.visit(context)
            if visit_future is not None:
                visit_future.reverse()
                walker.extend(visit_future)

                   
            if is_a(entity, ExecuteProcess):
                typed_action = cast(ExecuteProcess, entity)
                if typed_action.process_details is not None:
                    sys.stdout = sys.__stdout__
                    commands = []
                    for cmd in typed_action.process_details['cmd']:
                        if cmd.strip():
                            commands.extend(['"{}"'.format(cmd.strip())])
                    if os.sep not in typed_action.process_details['cmd'][0]:
                        commands[0] = '"{}"'.format(find_files(typed_action.process_details['cmd'][0]))
                    print(' '.join(commands))
                    sys.stdout = mystring

            # Lifecycle node support
            # https://github.com/ms-iot/vscode-ros/issues/632
            # Lifecycle nodes use a long running future to monitor the state of the nodes.
            # cancel this task, so that we can shut down the executor in ros_adapter
            async_future = entity.get_asyncio_future()
            if async_future is not None:
                async_future.cancel()
        
    except Exception as ex:
        print(ex, file=sys.stderr)

    # Shutdown the ROS Adapter 
    # so that long running tasks shut down correctly in the debug bootstrap scenario.
    # https://github.com/ms-iot/vscode-ros/issues/632
    ros_adapter = get_ros_adapter(context)
    if ros_adapter is not None:
        ros_adapter.shutdown()
