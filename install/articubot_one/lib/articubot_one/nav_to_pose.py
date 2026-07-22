#!/usr/bin/env python3

"""
ROS 2 Node to send Nav2 goal poses via a terminal menu interface.
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NavGoalMenuNode(Node):
    def __init__(self):
        super().__init__('nav_goal_menu')
        
        # Publisher to publish PoseStamped goals on the '/goal_pose' topic
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info('Nav Goal Menu Node has been started.')

    def publish_goal(self, x, y, name):
        """
        Constructs and publishes a PoseStamped goal message.
        """
        goal = PoseStamped()
        
        # Populate headers
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        
        # Set Position
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # Set Orientation (default orientation: yaw = 0, represented as quaternion w=1.0)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        
        # Publish
        self.publisher_.publish(goal)
        self.get_logger().info(f"Goal sent for {name}: x = {x:.4f}, y = {y:.4f}")

    def run_menu(self):
        """
        Runs the command-line menu interface.
        """
        destinations = {
            'A': ('Location A', 1.9950050115585327, 1.0249648094177246),
            'B': ('Location B', 1.9899433851242065, -0.9721394181251526),
            'C': ('Location C', -2.0015270709991455, 0.9685487151145935),
            'D': ('Location D', -1.9932698011398315, -1.028678297996521),
            'E': ('Home (Initial Position)', -0.012512493878602982, -0.008390679024159908)
        }
        
        while rclpy.ok():
            print("\nSelect a destination:")
            for key, (name, _, _) in sorted(destinations.items()):
                print(f"{key} - {name}")
            print("Q - Quit")
            
            try:
                # Raw input from terminal
                choice = input("\nEnter your choice: ").strip().upper()
            except EOFError:
                # Handle cases where stdin is closed or interrupted
                break
                
            if choice == 'Q':
                self.get_logger().info("Exiting...")
                break
            elif choice in destinations:
                name, x, y = destinations[choice]
                self.publish_goal(x, y, name)
            else:
                print(f"Error: Invalid option '{choice}'. Please select A, B, C, D, E, or Q.")

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalMenuNode()
    
    # Use a custom SingleThreadedExecutor to manage node execution
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    # Spin the executor in a background thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        node.run_menu()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly shutdown the executor and thread
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
