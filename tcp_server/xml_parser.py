#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import xml.etree.ElementTree as ET

class xml_parser(Node):
    def __init__(self):
        super().__init__("xml_parser")
        self.xml_subscriber = self.create_subscription(String,"/xml_data",self.xml_callback,0)
        self.publisher = self.create_publisher(String,'/parsed_xml',10)
    
    def xml_callback(self, msg:String):
        new_msg = String()
        self.get_logger().info("Received data. Parsing...")
        xml_string = msg.data
        root = ET.fromstring(xml_string)
        for child in root.iter():
            if child.text.strip():
                new_msg.data += f"{child.tag} : {child.text}"
        self.publisher.publish(new_msg)
        self.get_logger().info("Parsed data sent to TCP server")

def main():
    rclpy.init()
    node = xml_parser()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()