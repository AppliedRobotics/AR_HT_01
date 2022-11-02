import socket
import time

class manip_api():
    def __init__(self):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.manip_ip = ("192.168.2.106", 8888) # ip and port target robot
        # self.manip_ip = ("manipulator.local", 8888) # ip and port target robot

    def send_cmd(self, x, y, z, angle, effector, timeout):
        cmd2 = 'g_b:'+str(x)+':'+str(y)+':'+str(angle)+':'+str(z)+':'+str(effector)+'#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(timeout)

    def on_calibrate(self):
        cmd2 = 'm'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(5)
    
    def grab(self, markers):
        time.sleep(3)
        cmd2 = 'm'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(5)

        cmd2 = 'g_b:370:70:0:0:1#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(3)

        cmd2 = 'g_b:370:70:0:1:1#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(3)

        cmd2 = 'g_b:-300:100:0:0:1#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(2)

        cmd2 = 'g_b:-300:100:0:2:0#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(2)

        cmd2 = 'g_b:-300:100:0:0:0#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(2)

        cmd2 = '1'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(3)

    def put(self):
        cmd2 = 'g_b:-300:100:0:0:0#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(5)

        cmd2 = 'g_b:-300:100:0:2:1#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(2)

        cmd2 = 'g_b:-300:100:0:0:1#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(2)

        cmd2 = 'g_b:370:70:0:1:1#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(3)

        cmd2 = 'g_b:370:70:0:0:0#'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(3)

        cmd2 = '1'
        self.udp_socket.sendto(cmd2.encode(), self.manip_ip)
        time.sleep(3)

def main():
    manip = manip_api()

if __name__ == '__main__':
    main()