#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from gpiozero import LED
from gpiozero.exc import GPIOError


class HeadlightServiceNode(Node):
    def __init__(self):
        super().__init__('headlight_service_node')

        # Declare and read configurable GPIO pin parameter (default: BCM 18)
        self.declare_parameter('gpio_pin', 18)
        self.led_pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value

        # Initialize hardware using gpiozero (uses BCM pin numbering by default)
        try:
            self.led = LED(self.led_pin)
            self.led.off()
            self.get_logger().info(f'GPIO LED initialized successfully on BCM pin {self.led_pin}')
        except GPIOError as e:
            self.get_logger().error(f'Failed to initialize GPIO pin {self.led_pin}: {e}')
            raise

        # Create ROS 2 service named 'headlight' using std_srvs/srv/SetBool
        self.srv = self.create_service(
            SetBool,
            'headlight',
            self.headlight_callback
        )

        self.get_logger().info('Service /headlight ready. (Request format -> data: true [ON], data: false [OFF])')

    def headlight_callback(self, request: SetBool.Request, response: SetBool.Response):
        """
        Service callback handler.
        request.data == True  --> LED ON
        request.data == False --> LED OFF
        """
        if request.data:
            self.led.on()
            response.success = True
            response.message = f"Headlight switched ON (BCM Pin {self.led_pin})"
            self.get_logger().info("Service call received: Headlight switched ON")
        else:
            self.led.off()
            response.success = True
            response.message = f"Headlight switched OFF (BCM Pin {self.led_pin})"
            self.get_logger().info("Service call received: Headlight switched OFF")

        return response

    def destroy_node(self):
        # Clean up GPIO pins when shutting down the ROS node
        if hasattr(self, 'led') and self.led:
            self.led.off()
            self.led.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadlightServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()