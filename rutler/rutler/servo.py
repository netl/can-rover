import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import can
from functools import partial

class crawlerModule(Node):
    def __init__(self):
        super().__init__('crawler_module')

        #parameters
        self.declare_parameter("base_id", 0x10)
        self.declare_parameter("voltage_scale", 20/4096)
        self.declare_parameter("voltage_offset", 0)
        self.declare_parameter("current_scale", 40/4096)
        self.declare_parameter("current_offset", -10)
        for n in range(8): #servo channels
            self.declare_parameter(f"channel_{n}_scale", 1000)
            self.declare_parameter(f"channel_{n}_offset", 1500)

        #publisher
        self.outputs = ["voltage", "current"]
        self.publisherss = []
        for topic in self.outputs:
            self.publisherss.append(self.create_publisher(Float32, f"crawler/{topic}", 10))

        #subscriber
        self.subscriptionss = []
        for channel in range(8):
            self.subscriptionss.append(self.create_subscription(Float32, f"crawler/channel_{channel}", partial(self.listener_callback, channel), 10))

        #self.subscription # prevent unused variable warning

        #canbus
        base_id = self.get_parameter("base_id").value 
        canFilter = [{"can_id":0x7f8,"can_mask":base_id | 0x008, "extended":False}]
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000, can_filters=canFilter)

        #start loop
        self.timer = self.create_timer(0.1, self.can_listener_loop)

    #publish can data 
    def can_listener_loop(self):
        msg = self.bus.recv(timeout=0)
        if msg:
            channel = msg.arbitration_id & 0x007
            if channel not in range(len(self.outputs)):
                return

            value = int.from_bytes(msg.data, "big")
            
            #scale and offset
            topic = self.outputs[channel]
            scale = self.get_parameter(f"{topic}_scale").value
            offset = self.get_parameter(f"{topic}_offset").value
            result = value*scale+offset

            #publish
            output = Float32()
            output.data = result
            self.publisherss[channel].publish(output)
            self.get_logger().info(f"{topic}:{result}")
   
    #set servo channels 
    def listener_callback(self, channel, msg):
        a = msg.data
        scale = self.get_parameter(f"channel_{channel}_scale").value
        offset = self.get_parameter(f"channel_{channel}_offset").value
        result = int(a*scale+offset)

        #can address base
        base_id = self.get_parameter("base_id").value

        #create can message
        bus_msg = can.Message(is_extended_id=False)
        bus_msg.data = result.to_bytes(2,"big")
        bus_msg.arbitration_id = base_id | channel
        bus_msg.dlc = 2

        #send
        self.bus.send(bus_msg, timeout=0)
        self.get_logger().info(f"{channel}:{result}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(crawlerModule())

if __name__ == '__main__':
    main()
