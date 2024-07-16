import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from threading import Thread
from .Command import COMMAND as cmd
from .Thread import *
from .Video import *
from PyQt5.QtCore import *
from custom_robot_msgs.msg import RobotCommandMsgs
from functools import partial

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # self.subscription = self.create_subscription(
        #     String,
        #     'robot_commands',
        #     self.command_callback,
        #     10)
        
        # self.subscription  # prevent unused variable warning

        self.connections = []
        self.threads = []
        self.IPs = ["192.168.0.206"]  # 192.168.0.201, 192.168.0.206, 192.168.0.208, 192.168.0.204, 192.168.0.200, 192.168.0.209, 192.168.0.203, 192.168.0.205
        self.stop_event = threading.Event()
        self.TCP = VideoStreaming()
        self.robot_id_to_ip = {i: ip for i, ip in enumerate(self.IPs)}
        self.endChar = '\n'
        self.intervalChar = '#'
        global timer
   

    def get_TCP(self):
        return self.TCP
    
    def connect_agents(self):  
        robot_id = 0
        for ip in self.IPs:
            h = ip
            self.TCP.StartTcpClient(h)
            self.connections.append(h)
            try:
                streaming_thread = Thread(target=self.TCP.streaming, args=(h,))
                streaming_thread.start()
                self.threads.append(streaming_thread)
            except Exception as e:
                print(f'video error for {ip}: {e}')
            try:
                recv_thread = Thread(target=self.recvmassage, args=(h,))
                recv_thread.start()
                self.threads.append(recv_thread)
            except Exception as e:
                print(f'recv error for {ip}: {e}')

            #self.create_subscription(RobotCommandMsgs, f'/robot_{robot_id}/robot_commands', self.command_callback(robot_id), 10)
            self.create_subscription(
                RobotCommandMsgs,
                f'/robot_{robot_id}/robot_commands',
                partial(self.command_callback, robot_id=robot_id),
                10
            )
            robot_id += 1

        print('Connected to servers: ' + ', '.join(self.IPs) + '\n')


    def command_callback(self, msg, robot_id):
        linear_velocity = msg.linear_velocity
        angular_velocity = msg.angular_velocity
        x = msg.x
        y = msg.y
        tar_x = msg.x_target
        tar_y = msg.y_target
        ip = self.robot_id_to_ip[robot_id]
        
        # Translate the velocities into real commands
        self.translate_command(linear_velocity, angular_velocity, x, y, tar_x, tar_y, ip)


    def translate_command(self, linear_velocity, angular_velocity, x, y, ip):
        # Implement the logic to translate velocities to actual commands
        angular_velocity = int(angular_velocity * 10000)
        linear_velocity = int(linear_velocity * 10000)

        forward = self.intervalChar + str(linear_velocity) + self.intervalChar + str(linear_velocity) + self.intervalChar + str(
                angular_velocity) + self.intervalChar + str(angular_velocity) + self.endChar
        
        # forward = self.intervalChar + str(1500) + self.intervalChar + str(1500) + self.intervalChar + str(
        #         1500) + self.intervalChar + str(1500) + self.endChar

        command = cmd.CMD_MOTOR + forward #+ turn_command
        
        self.TCP.sendData(command)
        
    
    def disconnect_agents(self):
        self.stop_event.set() 
        try:
            for thread in self.threads:
                stop_thread(thread)
        except Exception as e:
            print(f'Error stopping threads: {e}')
        self.threads = []
        self.connections = []
        self.stop_event.clear()
        self.TCP.StopTcpcClient()

    def recvmassage(self, h):
        self.TCP.socket1_connect(h)
        self.power = Thread(target=self.Power)
        self.power.start()
        restCmd = ""

        while True:
            Alldata = restCmd + str(self.TCP.recvData())
            restCmd = ""
            print(Alldata)
            if Alldata == "":
                break
            else:
                cmdArray = Alldata.split("\n")
                if (cmdArray[-1] != ""):
                    restCmd = cmdArray[-1]
                    cmdArray = cmdArray[:-1]
            for oneCmd in cmdArray:
                Massage = oneCmd.split("#")
                if cmd.CMD_SONIC in Massage:
                    u = 'Obstruction:%s cm' % Massage[1]
                    self.U.send(u)
                elif cmd.CMD_LIGHT in Massage:
                    l = "Left:" + Massage[1] + 'V' + ' ' + "Right:" + Massage[2] + 'V'
                    self.L.send(l)
                #elif cmd.CMD_POWER in Massage:
                    #percent_power = int((float(Massage[1]) - 7) / 1.40 * 100)
                    #self.Pb.send(percent_power)


    def stop_threads(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join()  # Wait for threads to finish cleanly


    def Power(self):
        while True:
            try:
                self.TCP.sendData(cmd.CMD_POWER + self.endChar)
                time.sleep(60)
            except:
                break

def main(args=None):
    rclpy.init(args=args)
    robot_controller_node = RobotControllerNode()
    robot_controller_node.connect_agents()
    rclpy.spin(robot_controller_node)
    robot_controller_node.stop_threads()
    robot_controller_node.disconnect_agents()
    robot_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
