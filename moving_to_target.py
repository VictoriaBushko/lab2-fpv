import math

from typing import TYPE_CHECKING



from geometry_msgs.msg import TwistStamped



from ..flight_state import FlightState

from .handler import FlightStateHandler



if TYPE_CHECKING:

    from ..flight_test_node import FlightTestNode





class MovingToTargetHandler(FlightStateHandler):



    def handle(self, node: "FlightTestNode") -> None:

        if node.status is None or not node.status.armed:

            node.fail("Drone disarmed unexpectedly during movement")

            return



        if node.odometry is None:

            return



        current_x = node.odometry.pose.pose.position.x

        current_y = node.odometry.pose.pose.position.y



        if node.start_x is None or node.start_y is None:

            node.start_x = current_x

            node.start_y = current_y



            node.target_x = node.start_x + node.DELTA_X

            node.target_y = node.start_y + node.DELTA_Y



            node.get_logger().info(

                f"Start position: x={node.start_x:.2f}, "

                f"y={node.start_y:.2f}"

            )



            node.get_logger().info(

                f"Target position: x={node.target_x:.2f}, "

                f"y={node.target_y:.2f}"

            )



        error_x = node.target_x - current_x

        error_y = node.target_y - current_y



        distance = math.sqrt(

            error_x ** 2 + error_y ** 2

        )



        node.get_logger().info(

            f"Position: x={current_x:.2f}, "

            f"y={current_y:.2f}, "

            f"distance={distance:.2f} m"

        )



        if distance <= node.TARGET_TOLERANCE:

            self.stop_drone(node)



            node.get_logger().info(

                f"Target reached. Final error: "

                f"{distance:.2f} m"

            )



            node.state = FlightState.LANDING

            return



        speed = min(

            node.MAX_SPEED,

            distance

        )



        orientation = node.odometry.pose.pose.orientation



        sin_yaw = 2.0 * (

            orientation.w * orientation.z

            + orientation.x * orientation.y

        )



        cos_yaw = 1.0 - 2.0 * (

            orientation.y ** 2

            + orientation.z ** 2

        )



        current_yaw = math.atan2(

            sin_yaw,

            cos_yaw

        )



        target_yaw = math.atan2(

            error_y,

            error_x

        )



        yaw_error = target_yaw - current_yaw



        yaw_error = math.atan2(

            math.sin(yaw_error),

            math.cos(yaw_error)

        )



        max_yaw_rate = 0.5



        yaw_rate = max(

            -max_yaw_rate,

            min(max_yaw_rate, yaw_error)

        )



        command = TwistStamped()



        command.header.stamp = (

            node.get_clock().now().to_msg()

        )



        command.header.frame_id = "base_link"



        if abs(yaw_error) > 0.25:

            command.twist.linear.x = 0.0

        else:

            command.twist.linear.x = speed



        command.twist.linear.y = 0.0

        command.twist.linear.z = 0.0



        command.twist.angular.x = 0.0

        command.twist.angular.y = 0.0

        command.twist.angular.z = yaw_rate



        node.velocity_publisher.publish(command)



    def stop_drone(

        self,

        node: "FlightTestNode"

    ) -> None:



        command = TwistStamped()



        command.header.stamp = (

            node.get_clock().now().to_msg()

        )



        command.header.frame_id = "base_link"



        command.twist.linear.x = 0.0

        command.twist.linear.y = 0.0

        command.twist.linear.z = 0.0



        command.twist.angular.x = 0.0

        command.twist.angular.y = 0.0

        command.twist.angular.z = 0.0



        node.velocity_publisher.publish(command)

