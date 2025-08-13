# enhanced_terminal_bt_cli.py

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
import threading
from action_msgs.msg import GoalStatus

AVAILABLE_ROBOTS = ['george', 'fred', 'turtlebot4']
BUSY_ROBOTS = set()

ROOM_LOCATIONS = {
    'room_251': {'x': -4.5, 'y': 1.7, 'yaw': 0.0},
    'room_252': {'x': 1.3, 'y': 1.7, 'yaw': 0.0},
    'room_253': {'x': 4.57, 'y': 1.7, 'yaw': 0.0},
    'room_254': {'x': 4.57, 'y': -1.7, 'yaw': 0.0},
    'room_255': {'x': 1.3, 'y': -1.7, 'yaw': 0.0},
    'room_256': {'x': -1.3, 'y': -1.7, 'yaw': 0.0},
    'room_257': {'x': -4.5, 'y': -1.7, 'yaw': 0.0},
    'room_test': {'x': -0.5, 'y': -0.5, 'yaw': 0.0},
    'dock1': {'x': -2.25, 'y': 2.4, 'yaw': 0.0},
    'dock2': {'x': -2.55, 'y': 2.2, 'yaw': 0.0},
}

def timed_input(prompt, timeout=5):
    user_input = [None]

    def get_input():
        user_input[0] = input(prompt)

    thread = threading.Thread(target=get_input)
    thread.start()
    thread.join(timeout)
    if thread.is_alive():
        print("\nTimeout! No response received.")
        return None
    return user_input[0]

class DeliveryCommander(Node):
    def __init__(self):
        super().__init__('delivery_commander')

        available = [r for r in AVAILABLE_ROBOTS if r not in BUSY_ROBOTS]
        if not available:
            print("No available robots at the moment.")
            rclpy.shutdown()
            return

        print("Available robots:", ', '.join(available))
        self.namespace = input("Select a robot namespace from above: ")
        if self.namespace not in available:
            print("Invalid selection. Exiting.")
            rclpy.shutdown()
            return

        BUSY_ROBOTS.add(self.namespace)
        self.client = ActionClient(self, NavigateToPose, f'/{self.namespace}/navigate_to_pose')
        self.near_goal_counter = 0
        self.pickup_room = None
        self.start_delivery_cycle()

    def load_room_waypoint(self, room_name):
        if room_name not in ROOM_LOCATIONS:
            self.get_logger().error(f"Unknown room: {room_name}")
            return None
        return ROOM_LOCATIONS[room_name]

    def send_goal(self, wp):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        quat = quaternion_from_euler(0, 0, wp['yaw'])
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        #self.get_logger().info(f"Remaining distance: {distance:.2f} m")

        if distance < 1.0:
            self.near_goal_counter += 1
        else:
            self.near_goal_counter = 0

        if self.near_goal_counter > 5:
            self.get_logger().warn("Stuck near goal! Possibly blocked (e.g., door closed).")
            print("The robot is near your room but cannot reach the final position. Please open the door.")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result_msg = future.result()
        result = result_msg.result
        status = result_msg.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Robot has arrived at destination.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Navigation aborted! Possibly blocked.")
            print("Robot could not reach the destination. Possibly a closed door.")
        else:
            self.get_logger().error(f"Navigation ended with status code: {status}")

        self.continue_flow()

    def continue_flow(self):
        if self.state == 'pickup':
            confirm = timed_input("Sender: Please confirm that the package has been loaded onto the turtlebot with [y]: ", 5)
            if confirm and confirm.lower() == 'y':
                print("Available delivery rooms:", ', '.join([room for room in ROOM_LOCATIONS if room.startswith('room_')]))
                dest = input("Enter delivery room: ")
                self.target = self.load_room_waypoint(dest)
                if self.target:
                    self.state = 'deliver'
                    self.send_goal(self.target)
            else:
                self.get_logger().warn("No confirmation from sender. Returning to dock.")
                self.state = 'dock'
                self.send_goal(self.load_room_waypoint('dock1'))

        elif self.state == 'deliver':
            confirm = timed_input("Receiver: Please confirm that package was delivered and picked up with [y]: ", 5)
            if confirm and confirm.lower() == 'y':
                self.get_logger().info('Returning to dock station.')
                self.state = 'dock'
                self.send_goal(self.load_room_waypoint('dock1'))
            else:
                self.get_logger().warn("No response from receiver. Returning package to sender.")
                self.state = 'return_to_sender'
                self.send_goal(self.load_room_waypoint(self.pickup_room or 'room_test'))

        elif self.state == 'return_to_sender':
            confirm = timed_input("Sender: Please confirm that package was returned. [y]: ", 5)
            if confirm and confirm.lower() == 'y':
                self.get_logger().info("Package returned to sender successfully.")
                BUSY_ROBOTS.remove(self.namespace)
                self.shutdown_or_restart()
            else:
                self.get_logger().warn("Sender not responding. Informing system monitor.")
                self.state = 'to_monitor'
                self.send_goal(self.load_room_waypoint('room_251'))

        elif self.state == 'to_monitor' or self.state == 'dock':
            self.get_logger().info("Robot is docked or sent to monitor. Available for next task.")
            BUSY_ROBOTS.remove(self.namespace)
            self.shutdown_or_restart()

    def shutdown_or_restart(self):
        again = input("Do you want to send another delivery? [y/n]: ")
        if again.lower() == 'y':
            DeliveryCommander()
        else:
            rclpy.shutdown()

    def start_delivery_cycle(self):
        print("Available rooms:", ', '.join([room for room in ROOM_LOCATIONS if room.startswith('room_')]))
        room = input("Enter pickup room: ")
        self.pickup_room = room
        self.target = self.load_room_waypoint(room)
        if self.target:
            self.state = 'pickup'
            self.send_goal(self.target)

def main():
    rclpy.init()
    node = DeliveryCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
