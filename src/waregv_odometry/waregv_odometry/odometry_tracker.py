import math
from typing import Tuple

class OdometryTracker:
    # This class doesn't know anything about ROS. It just does the raw math
    # to figure out where the robot is based on wheel speeds and the IMU.
    
    def __init__(self):
        # We start at the origin (0,0) facing perfectly straight (angle 0).
        # x and y are our position in meters.
        # theta is our heading (which way we are facing) in radians.
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

    def update(self, 
               vel_left: float, 
               vel_right: float, 
               imu_yaw_rate: float, 
               wheel_radius: float, 
               dt: float) -> Tuple[float, float]:
        
        # 1. Figure out how fast we are moving straight forward (Linear Velocity).
        # We average the speed of the left and right wheels to get the center speed.
        # Multiplying by the wheel's radius turns the spin speed (rad/s) into actual physical speed (m/s).
        linear_velocity = (wheel_radius / 2.0) * (vel_right + vel_left)
        
        # 2. Figure out how fast we are spinning (Angular Velocity).
        # We could calculate this from the wheels, but wheels slip a lot on the ground.
        # The IMU's gyroscope is way more trustworthy for spinning, so we just use that directly.
        angular_velocity = imu_yaw_rate
        
        # 3. Figure out how much we turned during this tiny time step (dt).
        delta_theta = angular_velocity * dt
        
        # Here is a neat trick: Midpoint Integration.
        # If we calculate our new X/Y position using the angle we *started* at, it's slightly inaccurate.
        # Instead, we guess what our angle was exactly halfway through the movement.
        # This makes the robot's estimated path much smoother and more accurate.
        mid_theta = self.theta + (delta_theta / 2.0)
        
        # Now update our X and Y position on the map.
        # Basic trigonometry: cos() gives us the X direction, sin() gives us the Y direction.
        self.x += linear_velocity * math.cos(mid_theta) * dt
        self.y += linear_velocity * math.sin(mid_theta) * dt
        
        # Finally, update our actual heading with the full turn amount.
        self.theta += delta_theta
        
        # We need to keep theta between -180 and +180 degrees (-pi to pi).
        # If the robot spins in circles all day, we don't want the angle to grow to a million.
        # atan2(sin, cos) is just a math shortcut that instantly forces the angle back into that range.
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Send the calculated speeds back to the ROS node so it can broadcast them.
        return linear_velocity, angular_velocity