import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        # Parameter geometris robot berdasarkan gambar
        self.wheel_radius = 0.03       # Radius roda 3 cm
        self.wheel_separation = 0.17   # Jarak antar roda 17 cm
        
        # Subscribe ke topik cmd_vel (disesuaikan dengan mover_node.py, 
        # aslinya di gambar tertulis '/input_ik')
        self.create_subscription(
            Twist, '/cmd_vel', self.velocity_callback, 10
        )
        
        # Publisher untuk memberikan perintah kecepatan ke roda kiri dan kanan
        self.left_publisher = self.create_publisher(
            Float64, '/left_wheel/command', 10
        )
        self.right_publisher = self.create_publisher(
            Float64, '/right_wheel/command', 10
        )
        
        self.get_logger().info('Inverse kinematics aktif.')

    def velocity_callback(self, msg):
        # 1. Ambil target kecepatan linear (v) dan kecepatan sudut (omega) dari Twist
        v = msg.linear.x
        omega = msg.angular.z
        
        # 2. Ambil parameter robot
        R = self.wheel_radius
        L = self.wheel_separation
        
        # 3. Hitung Inverse Kinematics untuk masing-masing roda
        # Rumus:         
        w_left = (v - (omega * L / 2.0)) / R
        w_right = (v + (omega * L / 2.0)) / R
        
        # 4. Siapkan pesan Float64 dan publikasikan ke topik masing-masing roda
        left_msg = Float64()
        left_msg.data = w_left
        self.left_publisher.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = w_right
        self.right_publisher.publish(right_msg)
        
        # Opsional: Log kecepatan roda untuk debugging
        self.get_logger().info(f'v: {v:.2f}, w: {omega:.2f} -> WL: {w_left:.2f}, WR: {w_right:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
