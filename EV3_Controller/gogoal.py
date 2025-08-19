import math
import numpy as np
import matplotlib.pyplot as plt

class OmniRobot:
    def __init__(self, pos, ori):
        self.wheel_radius = 2.5/100
        self.robot_radius = 15/100 
        self.position = np.array(pos, dtype=float)
        self.orientation = ori
        self.max_yaw_velocity = 1.0
        self.max_wheel_speed = 15
        self.pat = [self.position.copy()]
        
    def move_to_target(self, target_position, max_speed=1.0, dt=0.1):
        """Move the robot towards a target position by using current position, orientation, and some differential inverse kinematics."""
        target_position = np.array(target_position)
        
        if not np.allclose(self.position, target_position, atol=0.01):
            # Calculate direction and distance
            direction = target_position - self.position
            distance = np.linalg.norm(direction)
            
            # Calculate desired linear velocity
            if distance > 0:
                linear_velocity = direction / distance * min(max_speed, distance / dt)
            else:
                linear_velocity = np.array([0.0, 0.0])
            
            # Calculate desired angular velocity for orientation control
            target_angle = math.atan2(direction[1], direction[0])
            angle_error = self.normalize_angle(target_angle - self.orientation)
            
            # Proportional control for angular velocity
            angular_velocity = np.clip(angle_error * 2.0, -self.max_yaw_velocity, self.max_yaw_velocity)
            
            # Combine into velocity vector
            velocity = np.array([linear_velocity[0], linear_velocity[1], angular_velocity])
            
            # Get wheel velocities
            wheel_velocities_percent = self.inverse_kinematics(velocity)
            
            # wheel_velocities = self.update_state(wheel_velocities_percent, dt)
            self.pat.append(self.position.copy())
            
            print(f"Current position: {self.position}, Target: {target_position}")
            print(f"Current orientation: {self.orientation:.3f}, Target angle: {target_angle:.3f}")
            print(f"Wheel velocities (%): {wheel_velocities_percent}")
            
            return wheel_velocities_percent
        else:
            return np.array([0.0, 0.0, 0.0])
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def inverse_kinematics(self, velocity):
        """Convert robot velocity to wheel velocities"""
        vx, vy, omega = velocity
        R = self.robot_radius
        
        # Jacobian matrix for 3-wheel omni robot (120-degree spacing)
        # From the Paper. Make sure to connect motors in same orientation as paper
        jacobian = (1/self.wheel_radius) * np.array([
            [-math.sin(self.orientation), math.cos(self.orientation), R],
            [-math.sin(self.orientation + 2*math.pi/3), math.cos(self.orientation + 2*math.pi/3), R],
            [-math.sin(self.orientation - 2*math.pi/3), math.cos(self.orientation - 2*math.pi/3), R]
        ])
        
        wheel_velocities = np.dot(jacobian, velocity)
        
        wheel_velocities_percent = (wheel_velocities / self.max_wheel_speed) * 100
        
        # Scale down if any wheel exceeds 100%
        max_percent = np.max(np.abs(wheel_velocities_percent))
        if max_percent > 100:
            wheel_velocities_percent = wheel_velocities_percent * (100 / max_percent)
            
        return wheel_velocities_percent
    
    # Required for Sim
    def update_state(self, wheel_velocities_percent, dt):
        """Update robot position and orientation based on wheel velocities"""
        # Convert percentage back to actual wheel velocities
        wheel_velocities = (wheel_velocities_percent / 100) * self.max_wheel_speed
        
        # Forward kinematics to get robot velocity
        robot_velocity = self.forward_kinematics(wheel_velocities)
        
        # Update position (convert local velocity to global frame)
        # local_velocity = robot_velocity[:2]
        # global_velocity = self.local_to_global(local_velocity)

        # self.position += global_velocity * dt
        
        # Update orientation
        omega = robot_velocity[2]
        # self.orientation += omega * dt
        # self.orientation = self.normalize_angle(self.orientation)
        
        return wheel_velocities
    
    # Required for Sim
    def forward_kinematics(self, wheel_velocities):
        """Convert wheel velocities to robot velocity in local frame"""
        w1, w2, w3 = wheel_velocities
        R = self.robot_radius
        r = self.wheel_radius
        
        # Velocity in robot's local frame
        vx = r * (-math.sin(0) * w1 - math.sin(0+2*math.pi/3) * w2 - math.sin(0-2*math.pi/3) * w3) / 3
        vy = r * (math.cos(0) * w1 + math.cos(0+2*math.pi/3) * w2 + math.cos(0-2*math.pi/3) * w3) / 3
        omega = r * (w1 + w2 + w3) / (3 * R)
        
        return np.array([vx, vy, omega])
    
    def global_to_local(self, global_vector):
        """Transform global coordinates to robot's local frame"""
        rotation_matrix = np.array([
            [math.cos(self.orientation), math.sin(self.orientation)],
            [-math.sin(self.orientation), math.cos(self.orientation)]
        ])
        return rotation_matrix.dot(global_vector)
    
    def local_to_global(self, local_vector):
        """Transform robot's local coordinates to global frame"""
        rotation_matrix = np.array([
            [math.cos(self.orientation), -math.sin(self.orientation)],
            [math.sin(self.orientation), math.cos(self.orientation)]
        ])
        return rotation_matrix.dot(local_vector)
    
    # For simulation
    def plot_path(self):
        """Plot the robot's path"""
        if len(self.pat) > 1:
            pat_array = np.array(self.pat)
            plt.figure(figsize=(10, 8))
            plt.plot(pat_array[:, 0], pat_array[:, 1], 'b-', linewidth=2, label='Robot Path')
            plt.plot(pat_array[0, 0], pat_array[0, 1], 'go', markersize=10, label='Start')
            plt.plot(pat_array[-1, 0], pat_array[-1, 1], 'ro', markersize=10, label='End')
            plt.grid(True)
            plt.axis('equal')
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.title('Robot Path')
            plt.legend()
            plt.show()
        else:
            print("Not enough points to plot path")

# Example usage:
if __name__ == "__main__":
    # Create robot at origin facing east
    robot = OmniRobot([0, 0], 0)
    
    # Target positions
    targets = [[1, 0], [1, 1], [0, 1], [0, 0]]
    
    # Move to each target
    for target in targets:
        print(f"\nMoving to target: {target}")
        steps = 0
        while not np.allclose(robot.position, target, atol=0.01) and steps < 100:
            robot.move_to_target(target, max_speed=0.5, dt=0.1)
            steps += 1
        print(f"Reached target in {steps} steps")
    
    # Plot the path
    robot.plot_path()
