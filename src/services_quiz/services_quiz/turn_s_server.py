#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from services_quiz_srv.srv import Turn

class TurnServer(Node):
    def __init__(self):
        super().__init__('turn_s_server')
        # Service must be called exactly "/turn"
        self.srv = self.create_service(Turn, '/turn', self.handle_turn)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Service [/turn] ready.')

    def handle_turn(self, request, response):
        direction = (request.direction or '').strip().lower()
        if direction not in ('left', 'right'):
            self.get_logger().error("direction must be 'left' or 'right'")
            response.success = False
            return response

        omega = float(abs(request.angular_velocity))
        duration = float(request.time)
        if omega <= 0.0 or duration <= 0.0:
            self.get_logger().error('angular_velocity and time must be > 0')
            response.success = False
            return response

        # Right = negative z, Left = positive z
        if direction == 'right':
            omega = -omega

        self.get_logger().info(
            f"Spinning {direction} @ {abs(omega):.3f} rad/s for {duration:.2f} s"
        )

        twist = Twist()
        twist.angular.z = omega

        end_t = time.monotonic() + duration
        try:
            while rclpy.ok() and time.monotonic() < end_t:
                self.pub.publish(twist)
                time.sleep(0.05)  # ~20 Hz
        finally:
            twist.angular.z = 0.0
            self.pub.publish(twist)

        response.success = True
        return response

def main():
    rclpy.init()
    node = TurnServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
