# enhanced_terminal_bt_cli.py
# Full CLI: send task, wait for arrival, confirm pickup/delivery using NavigateToPose BT

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler

AVAILABLE_ROBOTS = ['robot1', 'robot2', 'turtlebot4']
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
    'dock1': {'x': 2.55, 'y': 2.2, 'yaw': 0.0},
    'dock2': {'x': -2.25, 'y': -2.4, 'yaw': 0.0},
}

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
        self._send_goal_future = self.client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Robot has arrived at destination.")
        self.continue_flow()

    def continue_flow(self):
        if self.state == 'pickup':
            confirm = input("Confirm that package was picked up by thr robot [y]: ")
            if confirm.lower() == 'y':
                print("Available delivery rooms:", ', '.join([room for room in ROOM_LOCATIONS if room.startswith('room_')]))
                dest = input("Enter delivery room: ")
                self.target = self.load_room_waypoint(dest)
                if self.target:
                    self.state = 'deliver'
                    self.send_goal(self.target)
        elif self.state == 'deliver':
            confirm = input("Confirm that package was delivered and picked up by the receiver  [y]: ")
            if confirm.lower() == 'y':
                self.state = 'dock'
                self.target = self.set_goal('dock1')
                if self.target:
                    self.send_goal(self.target)
        elif self.state == 'dock':
            self.get_logger().info("Robot is docked and available for next task.")
            BUSY_ROBOTS.remove(self.namespace)
            again = input("Do you want to send another delivery? [y/n]: ")
            if again.lower() == 'y':
                DeliveryCommander()
            else:
                rclpy.shutdown()

    def start_delivery_cycle(self):
        print("Available rooms:", ', '.join([room for room in ROOM_LOCATIONS if room.startswith('room_')]))
        room = input("Enter pickup room: ")
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
