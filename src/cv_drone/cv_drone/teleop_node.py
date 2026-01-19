import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty, Bool, String
import threading
import sys
import termios
import tty



LIN_VEL_INCREMENT = 0.1
ANG_VEL_INCREMENT = 0.1
T_CMD_VEL = 'cmd_vel'
class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_node')

        # publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, T_CMD_VEL, 120)
        
        # separate thread to listen to keyboard inputs
        self.input_thread = threading.Thread(target=self.read_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def read_keyboard_input(self) -> None:
        """
        description: 
            - reads keyboard input as follows to update command velocity:
                w/s = translation forwards/backwards
                a/d = translation left/right
                i/k = translation up/down
                j/l = rotation left/right
                q = reset velocity to 0
        inputs:
            - None
        outputs:
            - None
        """
        cmd_vel = Twist()

        while rclpy.ok():
            vel_report_str = f"vx, vy, vz, rz :  {str(round(cmd_vel.linear.x, 2))},  {str(round(cmd_vel.linear.y, 2))}, {str(round(cmd_vel.linear.z, 2))}, {str(round(cmd_vel.angular.z, 2))}"
            print(vel_report_str)
            print(" ")
 
            key = self.get_key()

            # zero velocity
            if key.lower() == 'q':
                cmd_vel = Twist()
                self.cmd_vel_publisher.publish(cmd_vel)
                
            # Move forward
            elif key.lower() == 'w':
                
                cmd_vel.linear.x = cmd_vel.linear.x + LIN_VEL_INCREMENT 
                self.cmd_vel_publisher.publish(cmd_vel)

            #move backward
            elif key.lower() == 's':
                cmd_vel.linear.x = cmd_vel.linear.x - LIN_VEL_INCREMENT 
                self.cmd_vel_publisher.publish(cmd_vel)
                
            # Move Left
            elif key.lower() == 'a':
                cmd_vel.linear.y = cmd_vel.linear.y + LIN_VEL_INCREMENT
                self.cmd_vel_publisher.publish(cmd_vel)
            
            # Move right
            elif key.lower() == 'd':
                cmd_vel.linear.y =  cmd_vel.linear.y - LIN_VEL_INCREMENT
                self.cmd_vel_publisher.publish(cmd_vel)
            
            # Move up
            if key.lower() == 'i':    
                cmd_vel.linear.z =  cmd_vel.linear.z + LIN_VEL_INCREMENT 
                self.cmd_vel_publisher.publish(cmd_vel)
            
            #move down
            elif key.lower() == 'k':
                cmd_vel.linear.z =  cmd_vel.linear.z - LIN_VEL_INCREMENT 
                self.cmd_vel_publisher.publish(cmd_vel)
            
            # rotate left
            elif key.lower() == 'j':
                
                cmd_vel.angular.z = cmd_vel.angular.z + ANG_VEL_INCREMENT
                self.cmd_vel_publisher.publish(cmd_vel)

            # rotate right
            elif key.lower() == 'l':
                
                cmd_vel.angular.z = cmd_vel.angular.z - ANG_VEL_INCREMENT
                self.cmd_vel_publisher.publish(cmd_vel)
            
            # exit
            elif key == 'p':
                print("EXITING)")
                exit()           
            
    def get_key(self) -> str:
        """
        description: 
            - reads keyboard input in a non-blocking manner
        inputs:
            - None
        outputs:
            - None
        """

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()