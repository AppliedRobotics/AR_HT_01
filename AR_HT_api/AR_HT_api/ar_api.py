import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from time import sleep
import threading
class AR_HT_api(Node):
    def __init__(self):
        super().__init__('ar_api')
        self.lift_pub = self.create_publisher(Bool, 'lift', 1)
    def lift(self, state, sleep_time): #sleep time = 12 for full cycle of up and down
        msg = Bool()
        if state == True:
            msg.data = True
            self.lift_pub.publish(msg)
            sleep(sleep_time)
            return True
        elif state == False:
            msg.data = False
            self.lift_pub.publish(msg)
            sleep(sleep_time)
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    api = AR_HT_api()
    thread = threading.Thread(target=rclpy.spin, args=(api, ), daemon=True)
    thread.start()
    rate = api.create_rate(2)
    # future = action_client.send_goal(-1.85,-1.12,5.1)
    while rclpy.ok():
        try:
            api.lift(True)
            print("up")
            api.lift(False)
            print("down")
            rate.sleep()
        except KeyboardInterrupt:
            rclpy.shutdown()



if __name__ == '__main__':
    main()
