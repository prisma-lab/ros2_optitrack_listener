import rclpy
from rclpy.node import Node
from threading import Thread
import optirx as rx
import socket
from geometry_msgs.msg import PoseStamped
import numpy as np

class OptitrackListener( Node ):
    def __init__(self):
        super().__init__('optitrack_listener')
        self.declare_parameter('local_interface',  "192.168.1.13")
        self.ipaddr = self.get_parameter('local_interface').get_parameter_value().string_value
        
        self.thread = Thread(target = self.get_optitrack_data, args = ())
        self.thread.start()

    def get_optitrack_data(self):
        
        print("Using ipaddr: ", self.ipaddr)
        self.version = (2, 5, 0, 0)  # the latest SDK version
        self.optitrack = rx.mkdatasock(ip_address=self.ipaddr)#(ip_address=get_ip_address(iface))
        print('Successfully connected to optitrack\n')
        
        names = []
        ids = []
                
        first = True
        while rclpy.ok():
            try:
                data = self.optitrack.recv(rx.MAX_PACKETSIZE)
            except socket.error:
                
                print('Failed to receive packet from optitrack')
      
            packet = rx.unpack(data, version=self.version)
            print(packet)
            if first == True:
                print('NatNet version received: ' + str(self.version))
                first = False
            if type(packet) is rx.SenderData:
                self.version = packet.natnet_version
                print('NatNet version received: ' + str(self.version))

            if type(packet) in [rx.SenderData, rx.ModelDefs, rx.FrameOfData]:
                # Optitrack gives the position of the centroid.
                print("If")
                '''
                array_msg = RigidBodyArray()
                '''
                for i, rigid_body in enumerate(packet.rigid_bodies):
                    body_id = rigid_body.id
                    pos_opt = np.array(rigid_body.position)
                    rot_opt = np.array(rigid_body.orientation)

                    print( body_id )
                    print( pos_opt )
                    print( rot_opt )
                    print( rigid_body.tracking_valid )
                    print( rigid_body.mrk_mean_error )
                    print("Marker size: ", len(rigid_body.markers ))
                    print( rigid_body.markers )


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
