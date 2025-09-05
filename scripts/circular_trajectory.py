#!/usr/bin/env python3

import rospy
import math
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header

class CircularTrajectory:
    def __init__(self):
        rospy.init_node('circular_trajectory', anonymous=True)
        
        # Publisher for position commands
        self.cmd_pub = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        
        # Trajectory parameters
        self.radius = rospy.get_param('~radius', 2.0)  # Circle radius in meters
        self.height = rospy.get_param('~height', 1.5)  # Flight height in meters
        self.angular_velocity = rospy.get_param('~angular_velocity', 0.5)  # rad/s
        self.center_x = rospy.get_param('~center_x', 0.0)
        self.center_y = rospy.get_param('~center_y', 0.0)
        
        # Control parameters
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)  # Hz
        
        # Trajectory state
        self.start_time = None
        
        rospy.loginfo(f"Circular trajectory node initialized:")
        rospy.loginfo(f"  Radius: {self.radius}m")
        rospy.loginfo(f"  Height: {self.height}m") 
        rospy.loginfo(f"  Angular velocity: {self.angular_velocity} rad/s")
        rospy.loginfo(f"  Center: ({self.center_x}, {self.center_y})")
        rospy.loginfo(f"  Publish rate: {self.publish_rate} Hz")
    
    def generate_circular_command(self, t):
        """Generate circular trajectory command at time t"""
        
        # Angular position
        theta = self.angular_velocity * t
        
        # Position (circular motion in xy plane)
        x = self.center_x + self.radius * math.cos(theta)
        y = self.center_y + self.radius * math.sin(theta)
        z = self.height
        
        # Velocity (tangent to circle)
        vx = -self.radius * self.angular_velocity * math.sin(theta)
        vy = self.radius * self.angular_velocity * math.cos(theta)
        vz = 0.0
        
        # Acceleration (centripetal acceleration toward center)
        ax = -self.radius * self.angular_velocity**2 * math.cos(theta)
        ay = -self.radius * self.angular_velocity**2 * math.sin(theta)
        az = 0.0
        
        # Jerk (derivative of acceleration)
        jx = self.radius * self.angular_velocity**3 * math.sin(theta)
        jy = -self.radius * self.angular_velocity**3 * math.cos(theta)
        jz = 0.0
        
        # Yaw (tangent to velocity vector)
        yaw = math.atan2(vy, vx)
        yaw_dot = self.angular_velocity
        
        # Create PositionCommand message
        cmd = PositionCommand()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"
        
        cmd.position = Point(x, y, z)
        cmd.velocity = Vector3(vx, vy, vz)
        cmd.acceleration = Vector3(ax, ay, az)
        cmd.jerk = Vector3(jx, jy, jz)
        cmd.yaw = yaw
        cmd.yaw_dot = yaw_dot
        
        # Control gains (typical values)
        cmd.kx = [5.7, 5.7, 6.2]
        cmd.kv = [3.4, 3.4, 4.0]
        
        cmd.trajectory_id = 1
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        
        return cmd
    
    def run(self):
        """Main execution loop"""
        rate = rospy.Rate(self.publish_rate)
        
        # Wait for subscribers
        while self.cmd_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for subscribers...")
            rospy.sleep(1.0)
        
        rospy.loginfo("Starting circular trajectory execution...")
        self.start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Calculate elapsed time
            current_time = rospy.Time.now()
            if self.start_time is None:
                self.start_time = current_time
            
            elapsed_time = (current_time - self.start_time).to_sec()
            
            # Generate and publish command
            cmd = self.generate_circular_command(elapsed_time)
            self.cmd_pub.publish(cmd)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        circular_traj = CircularTrajectory()
        circular_traj.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Circular trajectory node terminated.")