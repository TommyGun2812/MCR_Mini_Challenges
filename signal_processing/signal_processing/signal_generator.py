import rclpy
import numpy as np
from numpy import pi
from rclpy.node import Node
from std_msgs.msg import Float32

class SignalGenerator(Node): 
    def __init__(self):
        super().__init__('signal_generator')
        # Creating publishers
        self.time_pub = self.create_publisher(Float32, 'time', 10)
        self.signal_pub = self.create_publisher(Float32, 'signal', 10)
        #Stablishing timer
        self.timer = self.create_timer(0.1, self.signal_cb)
        # Global classes
        self.time = Float32()
        self.signal = Float32()

    def signal_cb(self):
        # Values
        current_time = self.get_clock().now().seconds_nanoseconds()
        current_time_sec = current_time[0] + current_time[1] / 1e9 
        sinoidal = np.sin(0.5*pi*current_time_sec)
        # ROS mesasges & data
        self.time.data = float(current_time_sec)
        self.signal.data = sinoidal
        # Publishing on topics
        self.time_pub.publish(self.time)
        self.signal_pub.publish(self.signal)
        # Showing info on terminal
        self.get_logger().info(f'Time: {current_time_sec:.2f} s, Signal: {sinoidal:.4f}')

def main(args=None):
    #Stablish ROS ommunications
    rclpy.init(args=args)
    # Instance of class SignalGenerator()
    signal_generator = SignalGenerator()

    try:
        rclpy.spin(signal_generator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    signal_generator.destroy_node()
    rclpy.shutdown()

if __name__=='__main__': 
    main() 




