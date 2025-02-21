import rclpy
import numpy as np
from numpy import pi
from rclpy.node import Node
from std_msgs.msg import Float32

class Process(Node): 
    def __init__(self): 
        super().__init__('process')
        # Creating subscribers & publisher
        self.signal_sub = self.create_subscription(Float32, 'signal', self.sig_cb,10)
        self.time_sub = self.create_subscription(Float32, 'time', self.time_cb, 10)
        self.psignal_pub = self.create_publisher(Float32, 'proc_signal',10)
        # Stablish timer & frequency rate
        self.frequency_rate = 0.1
        self.timer = self.create_timer(self.frequency_rate, self.psig_cb)
        # Global variables & classes
        self.signal = Float32()
        self.time = Float32()
        self.proc_signal = Float32()
        self.offset = 1.0
        self.amplitude = 0.5
        self.shift = pi/2
        self.cos = 0.0
        self.last_data = 0.0

    def sig_cb(self, msg):
          # Signal data, last value and actual
          self.last_data = self.signal.data
          self.signal = msg
    

    def time_cb(self, msg):
         # Receiving time data
         self.time = msg

    def psig_cb(self):
        # Calculating cosine from original signal
        self.cos = np.sqrt(1 - self.signal.data**2)
        # Derivative from the 
        derivative = (self.signal.data - self.last_data)/self.frequency_rate
        # Derivative realtive value to ensure correct cosine value
        if derivative >= 0.0:
            self.cos *= -1.0
        # Processing the signal
        self.proc_signal.data = self.amplitude*(self.signal.data*np.cos(self.shift) + self.cos*np.sin(self.shift))
        self.proc_signal.data += self.offset
        self.psignal_pub.publish(self.proc_signal)
        self.get_logger().info(f'Signal received: {self.signal.data} | Processed signal: {self.proc_signal.data}')

def main(args=None):
    #Stablish ROS ommunications
    rclpy.init(args=args)
    # Instance of class SignalGenerator()
    process_node = Process()
    
    try:
        rclpy.spin(process_node)
    except KeyboardInterrupt:
        pass
    finally:
        process_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()