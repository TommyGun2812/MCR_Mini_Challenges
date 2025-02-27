'''Imports'''
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

class Controller(Node):
    def __init__(self): 
        super().__init__('control_node')
        '''Service initial conditions'''
        self.simulation_running = False
        '''Service instance'''
        self.service = self.create_service(SetProcessBool, 'SERVICE', self.service_cb)
        '''Declaring subscribers and publishers'''
        self.sp_sub = self.create_subscription(Float32, 'set_point', self.sp_cb, 10)
        self.motor_sub = self.create_subscription(Float32, 'motor_output_y', self.motor_cb, 10)
        self.motor_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        '''Creating a timer and its rate'''
        self.timer_rate = 0.1
        self.timer = self.create_timer(self.timer_rate, self.control_cb)
        '''Declaring parameters'''
        self.declare_parameter('Kp', 0.0) # Proportional gain
        self.declare_parameter('Ki', 0.0) # Integral gain
        self.declare_parameter('Kd', 0.0) # Derivative gain
        '''Get paramters initial values'''
        self.param_Kp = self.get_parameter('Kp').value
        self.param_Ki = self.get_parameter('Ki').value
        self.param_Kd = self.get_parameter('Kd').value       
        '''Declaring object of sending/receiving classes'''
        self.sp_signal = Float32() # Set point signal
        self.fb_signal = Float32() # Feedback
        self.ctrl_signal = Float32() # Control signal
        '''Initializing control signal variable'''
        self.ctrl_signal.data = 0.0 
        self.last_ctrl_signal = 0.0
        '''Initializing error value'''
        self.error = 0.0
        self.last_error = 0.0
        '''Parameters callback'''
        self.add_on_set_parameters_callback(self.prmtr_cb)
        '''Bash message'''
        self.get_logger().info('Controller Node Started \U0001F680')

    '''Control signal callback'''
    def control_cb(self):
        if not self.simulation_running:
            return
        '''Discretizing PID gains'''
        self.K1 = self.param_Kp + self.timer_rate * self.param_Ki + (self.param_Kd/self.timer_rate)
        self.K2 = -self.param_Kp - 2.0 * (self.param_Kd/self.timer_rate)
        self.K3 = self.param_Kd/self.timer_rate
        '''Computing error'''
        self.error = self.sp_signal.data - self.fb_signal.data
        '''Computing control signal with PID controller'''
        self.ctrl_signal.data = (self.K1 * self.error + self.K2 * self.last_error + self.K3 * self.error + self.last_ctrl_signal)
        '''Publishing control signal on motor'''
        self.motor_pub.publish(self.ctrl_signal)
        '''Storing actual error as previous one'''
        self.last_error = self.error
        '''Storing actual control signal as last one'''
        self.last_ctrl_signal = self.ctrl_signal.data

    '''Parameter callback'''
    def prmtr_cb(self,params): 
        for param in params:
            if param.name == "Kp": # Ensure Kp is not negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid Kp! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="Kp cannot be negative")
                else:
                    self.param_Kp = param.value  # Update internal variable
                    self.get_logger().info(f"Kp updated to {self.param_Kp}")

            if param.name == "Ki":
                if (param.value < 0.0):   # Ensure Ki is not negative
                    self.get_logger().warn("Invalid Ki! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="Ki cannot be negative")
                else:
                    self.param_Ki = param.value  # Update internal variable
                    self.get_logger().info(f"Ki updated to {self.param_Ki}")

            if param.name == "Kd":
                if (param.value < 0.0):  # Ensure Kd is not negative
                    self.get_logger().warn("Invalid Kd! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="Kd cannot be negative")
                else:
                    self.param_Kd = param.value  # Update internal variable
                    self.get_logger().info(f"Kd updated to {self.param_Kd}")
        return SetParametersResult(successful=True)

    '''Service callback'''
    def service_cb(self,request,response):
        '''Enable Service'''
        if request.enable:
            self.simulation_running = True
            self.get_logger().info("🚀 Simulation Started")
            '''Enable messages'''
            response.success = True
            response.message = "Simulation Started Successfully"
        else:
            '''Not enabling service'''
            self.simulation_running = False
            self.get_logger().info("🛑 Simulation Stopped")
            '''Not enabling messages'''
            response.success = True
            response.message = "Simulation Stoped Successfully"
        
        return response


    '''Set point callback'''
    def sp_cb(self, sp_signal):
        self.sp_signal = sp_signal

    '''Feedback callback'''
    def motor_cb(self, fb_signal):
        self.fb_signal = fb_signal
    

'''Definin main function'''
def main(args=None):
    '''Starting ros communications'''
    rclpy.init(args=args)
    '''Instantiating Controller object'''
    controller = Controller()
    '''Node execution template'''
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        '''Node destruction'''
        controller.destroy_node()
        rclpy.try_shutdown()

'''Execute controller node'''
if __name__ == '__main__':
    main()
