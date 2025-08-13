#!/usr/bin/env python3

# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Hilary Luo (hluo@clearpathrobotics.com)

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main(args=None):
    rclpy.init(args=args)

    navigator = TurtleBot4Navigator()

    # Start on dock
    navigator.info("Checking docked status...")
    docked = navigator.getDockedStatus()
    navigator.info(f"Docked status: {docked}")
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    print("Initializing robot...")
    initial_pose = navigator.getPoseStamped([-2.25, -2.4], TurtleBot4Directions.WEST)
    print("Setting pose...")
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    print("waiting Until Nav2Active")
    navigator.waitUntilNav2Active()

    # Undock
    print("undocking")
    navigator.undock()

    # Prepare goal pose options
    goal_options = [
        {'name': 'room_251',
         'pose': navigator.getPoseStamped([-4.5, 1.7], TurtleBot4Directions.NORTH)},

        {'name': 'room_252',
         'pose': navigator.getPoseStamped([1.3, 1.7], TurtleBot4Directions.NORTH)},

        {'name': 'room_253',
         'pose': navigator.getPoseStamped([4.57, 1.7], TurtleBot4Directions.NORTH)},

        {'name': 'room_254',
         'pose': navigator.getPoseStamped([4.57, -1.7], TurtleBot4Directions.SOUTH)},

        {'name': 'room_255',
         'pose': navigator.getPoseStamped([1.3, -1.7], TurtleBot4Directions.SOUTH)},

         {'name': 'room_256',
         'pose': navigator.getPoseStamped([-1.3, -1.7], TurtleBot4Directions.SOUTH)},

         {'name': 'room_257',
         'pose': navigator.getPoseStamped([-4.5, -1.7], TurtleBot4Directions.SOUTH)},

         {'name': 'room_test',
         'pose': navigator.getPoseStamped([-0.2, -0.2], TurtleBot4Directions.EAST)},

        {'name': 'Exit',
         'pose': None}
    ]

    navigator.info('Welcome to the mail delivery service.')
    # TODO in later updates, the free and next robot to the pickup location should accept the delivery task
    while True:
        # Create a list of the goals for display
        options_str = 'Please enter pickup room: \n'
        for i in range(len(goal_options)):
            options_str += f'    {i}. {goal_options[i]["name"]}\n'

        # Prompt the user for the goal location
        raw_input = input(f'{options_str}Selection: ')

        selected_index = 0

        # Verify that the value input is a number
        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'Invalid goal selection: {raw_input}')
            continue

        # Verify that the user input is within a valid range
        if (selected_index < 0) or (selected_index >= len(goal_options)):
            navigator.error(f'Goal selection out of bounds: {selected_index}')

        # Check for exit
        elif goal_options[selected_index]['name'] == 'Exit':
            break

        else:
            # Navigate to requested position
            navigator.startToPose(goal_options[selected_index]['pose'])

    rclpy.shutdown()


if __name__ == '__main__':
    main()
