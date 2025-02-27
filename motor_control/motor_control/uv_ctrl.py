import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult as SPR
#from custom_interfaces.srv import SetProcessBool as SPB

#PONERLE UN SERVICE
class ctrl(Node):
    def _init_(self):
        super()._init_('ctrl')

        self.simulation_running= False

        self.servere         = self.create_service(SPB,'SERVER',self.server_cb)
        self.motor_input_sub = self.create_subscription(Float32, 'set_point', self.point_callback,10) #Declarar Suscriptores y Publishers
        self.motor_input_sub = self.create_subscription(Float32, 'motor_output_y', self.motor_callback,10)
        self.motor_speed_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        self.timer           = self.create_timer(0.1, self.timer_cb)                                   #Declarar el Timer para enviar informacion

        self.declare_parameter('Kp',1.0) #Declarar Kp, Ki, Kd como parametros de entrada
        self.declare_parameter('Ki',1.0)
        self.declare_parameter('Kd',1.0)

        self.param_Kp = self.get_parameter('Kp').value #Declarar variables del nodo en la que se guardan los parametros
        self.param_Ki = self.get_parameter('Ki').value
        self.param_Kd = self.get_parameter('Kd').value

        self.output_msg = Float32() #Declarar Variable de salidad para publisher
        self.s          = Float32() #Declarar Variables reseptoras de data subscripotara
        self.y          = Float32()
        self.output_msg.data = 0.0 #inicializar la salida como 0
        self.out1       = 0.0      #inicializar variables de calculo como 0
        self.e          = 0.0
        self.e1         = 0.0

        self.add_on_set_parameters_callback(self.Parameter_cb)

        self.get_logger().info('Dynamical System Node Started \U0001F680')   
        
    #Timer Callback
    def timer_cb(self):
        if not self.simulation_running:
            return
        
        self.K1         = self.param_Kp + 0.1 * self.param_Ki + (self.param_Kd/0.1) #Calcular Ks de metodo PID discretisado 
        self.K2         = -self.param_Kp - 2.0 * (self.param_Kd/0.1)
        self.K3         = self.param_Kd/0.1
        self.e = self.s.data - self.y.data #calcular el error
        self.output_msg.data = (self.K1 * self.e + self.K2 * self.e1 + self.K3 * self.e + self.out1) #calcular señal de PID para el motor
        self.motor_speed_pub.publish(self.output_msg) #Publicar
        self.e1= self.e #Guardar el error anterior
        self.out1= self.output_msg.data #Guardar output anteriro de PID

        
    #Subscriber Callback
    def point_callback(self, input_s): #Callback de señal (Referencia)
        self.s = input_s

    def motor_callback(self, input_y): #Callback de output de Motor
        self.y = input_y

    def Parameter_cb(self,params):     #Callback de parametro Kp,Ki,Kd
        for param in params:
            if param.name == "Kp":
                if (param.value < 0.0): #Asegurar de que Kp no sea negativo
                    self.get_logger().warn("Invalid Kp! It cannot be negative.")
                    return SPR(successful=False, reason="Kp cannot be negative")
                else:
                    self.param_Kp = param.value  # Update internal variable
                    self.get_logger().info(f"Kp updated to {self.param_Kp}")

            if param.name == "Ki":
                if (param.value < 0.0):   #Asegurar de que Ki no sea negativo
                    self.get_logger().warn("Invalid Ki! It cannot be negative.")
                    return SPR(successful=False, reason="Ki cannot be negative")
                else:
                    self.param_Ki = param.value  # Update internal variable
                    self.get_logger().info(f"Ki updated to {self.param_Ki}")

            if param.name == "Kd":
                if (param.value < 0.0):  #Asegurar de que Kd no sea negativo
                    self.get_logger().warn("Invalid Kd! It cannot be negative.")
                    return SPR(successful=False, reason="Kd cannot be negative")
                else:
                    self.param_Kd = param.value  # Update internal variable
                    self.get_logger().info(f"Kd updated to {self.param_Kd}")
        return SPR(successful=True)

    def server_cb(self,request,response):
        if request.enable:
            self.simulation_running = True
            self.get_logger().info("🚀 Simulation Started")
            response.success = True
            response.message = "Simulation Started Successfully"
        else:
            self.simulation_running = False
            self.get_logger().info("🛑 Simulation Stopped")
            response.success = True
            response.message = "Simulation Stoped Successfully"

        return response

#Main
def main(args=None):
    rclpy.init(args=args)

    node = ctrl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if _name_ == '_main_':
    main()