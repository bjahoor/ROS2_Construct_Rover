#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from services_quiz_srv.srv import Turn

def main():
    rclpy.init()
    node = Node('turn_s_client')

    cli = node.create_client(Turn, '/turn')
    node.get_logger().info('Waiting for /turn service...')
    cli.wait_for_service()
    node.get_logger().info('/turn available. Sending request...')

    req = Turn.Request()
    req.direction = 'right'
    req.angular_velocity = 0.2
    req.time = 10.0

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    try:
        resp = future.result()
        node.get_logger().info(f"Service responded: success={resp.success}")
    except Exception as e:
        node.get_logger().error(f"Service call failed: {e}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
