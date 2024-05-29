#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
from std_msgs.msg import String
import time as t

class tcp_server(Node):
    def __init__(self):
        super().__init__("tcp_server")

        self.publisher = self.create_publisher(String,'/xml_data',10)
        self.xml_subscriber = self.create_subscription(String,"/parsed_xml",self.send_process_time,10)

        # Create a socket object
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Get the local machine name
        self.host = "172.20.66.29"
        self.port = 53241

        # Bind to the port
        self.server_socket.bind((self.host, self.port))
        self.get_logger().info(f"Local IP: {self.host}")
        self.get_logger().info(f"Server listening on port: {self.port}")

        # Connect to client
        self.connect_client()
    
    def connect_client(self):
        # Listen for incoming connections
        self.get_logger().info("Waiting PLC to connect")
        self.server_socket.listen(5)

        self.client_socket, self.client_address = self.server_socket.accept()
        self.get_logger().info(f"Accepted connection from {self.client_address}")

        # Receive data from client
        self.receive_data()

    def receive_data(self):
        self.get_logger().info("Waiting to recieve data")
        try:
            data = self.client_socket.recv(2048).decode()

            if not data:
                self.get_logger().info("Connection broken. Error: not data")
                return
                #self.connect_client()
        
        except Exception as error:
            self.get_logger().info(f"Connection broken. Error: {error}")
            self.connect_client()
        
        self.start_time = t.time()

        self.parse_data(data)
    
    def parse_data(self,data):
        msg = String()
        msg.data = data.replace("|","")
        self.get_logger().info("Sending data to parser")
        self.publisher.publish(msg)
        self.get_logger().info("Waiting for data to be parsed")

    def send_process_time(self,msg:String):
        self.get_logger().info(f"Data parsed: {msg.data}")
        self.get_logger().info("Sending process time to PLC")
        self.end_time = t.time()
        process_time = str(self.end_time - self.start_time)
        self.get_logger().info(f"Process time: {process_time}")
        self.client_socket.send(process_time.encode())
        self.receive_data()

def main():
    rclpy.init()
    node = tcp_server()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()