#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

# Attempt to import RPi.GPIO; fallback gracefully if running on non-Pi hardware
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class HeadlightServiceNode(Node):
    def __init__(self):
        super().__init__('headlight_service_node')
        global HAS_GPIO

        # Declare and read configurable GPIO pin parameter (default: BCM 18)
        self.declare_parameter('gpio_pin', 18)
        self.led_pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value

        # Initialize hardware if available
        if HAS_GPIO:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.led_pin, GPIO.OUT)
                GPIO.output(self.led_pin, GPIO.LOW)
                self.get_logger().info(f'GPIO initialized on BCM pin {self.led_pin}')
            except RuntimeError as e:
                self.get_logger().error(f'Failed to initialize GPIO: {e}. Running in simulation mode.')
                HAS_GPIO = False
        else:
            self.get_logger().warn('RPi.GPIO library not found or disabled. Running node in hardware simulation mode.')

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
            if HAS_GPIO:
                GPIO.output(self.led_pin, GPIO.HIGH)
            
            response.success = True
            response.message = f"Headlight switched ON (BCM Pin {self.led_pin})"
            self.get_logger().info("Service call received: Headlight switched ON")
        else:
            if HAS_GPIO:
                GPIO.output(self.led_pin, GPIO.LOW)
            
            response.success = True
            response.message = f"Headlight switched OFF (BCM Pin {self.led_pin})"
            self.get_logger().info("Service call received: Headlight switched OFF")

        return response

    def destroy_node(self):
        # Clean up GPIO pins when shutting down the ROS node
        if HAS_GPIO:
            GPIO.cleanup()
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