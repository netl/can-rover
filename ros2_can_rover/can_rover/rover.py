import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import can
from functools import partial
import time

class canRover(Node):
    def __init__(self):
        super().__init__('can_rover')

        #power monitoring
        self.power = {
            "voltage":0,
            "current":0,
            "watts":0,
            "wattHours":0,
            "time":time.time(),
        }

        #parameters
        self.declare_parameter("base_id", 0x10)
        self.declare_parameter("voltage_scale", 20./4096)
        self.declare_parameter("voltage_offset", 0.)
        self.declare_parameter("current_scale", -40./4096)
        self.declare_parameter("current_offset", 10.)
        for n in range(8): #servo channels
            self.declare_parameter(f"channel_{n+1}_scale", 1000)
            self.declare_parameter(f"channel_{n+1}_offset", 1500)

        #publisher
        self.outputs = ["voltage", "current", "watts", "wattHours"]
        self.publishTopics = []
        for topic in self.outputs:
            self.publishTopics.append(self.create_publisher(Float32, f"rover/{topic}", 10))

        #subscriber
        self.subscribeTopics = []
        for channel in range(8):
            self.subscribeTopics.append(self.create_subscription(Float32, f"rover/channel_{channel+1}", partial(self.listener_callback, channel), 10))

        self.setWh = self.create_subscription(Float32,  "rover/set_wattHours", self.set_wattHours, 10)

        #self.subscription # prevent unused variable warning

        #canbus
        base_id = self.get_parameter("base_id").get_parameter_value().integer_value
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
            scale = self.get_parameter(f"{topic}_scale").get_parameter_value().double_value
            offset = self.get_parameter(f"{topic}_offset").get_parameter_value().double_value
            result = value*scale+offset

            #publish
            output = Float32()
            output.data = result
            self.publishTopics[channel].publish(output)
            self.get_logger().debug(f"{topic}:{result}")

            #update power
            self.power.update({topic:result})
            t = time.time()
            w = self.power["voltage"]*self.power["current"]
            self.get_logger().debug(f"watts:{w}")

            #watt hours
            td = (t-self.power["time"])/3600 #time delta in hours
            wt = td*(w+self.power["watts"])/2 #trapezoidal
            wattHours = self.power["wattHours"]+wt
            self.get_logger().debug(f"wattHours:{wattHours}")
            self.power.update({"time":t,"wattHours":wattHours,"watts":w})

            #send
            wOut = Float32()
            wOut.data = w
            wHOut = Float32()
            wHOut.data = wattHours
            self.publishTopics[2].publish(wOut)
            self.publishTopics[3].publish(wHOut)
   
    #set servo channels 
    def listener_callback(self, channel, msg):
        a = msg.data
        scale = self.get_parameter(f"channel_{channel+1}_scale").get_parameter_value().integer_value
        offset = self.get_parameter(f"channel_{channel+1}_offset").get_parameter_value().integer_value
        result = int(a*scale+offset)

        #can address base
        base_id = self.get_parameter("base_id").get_parameter_value().integer_value

        #create can message
        bus_msg = can.Message(is_extended_id=False)
        bus_msg.data = result.to_bytes(2,"big")
        bus_msg.arbitration_id = base_id | channel
        bus_msg.dlc = 2

        #send
        self.bus.send(bus_msg, timeout=0)
        self.get_logger().debug(f"{channel}:{result}")

    def set_wattHours(self, msg):
        self.power.update({"wattHours":msg.data})

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(canRover())

if __name__ == '__main__':
    main()
