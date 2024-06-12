Dependencies:

apt-get install python3-pip
pip3 install optirx

Streaming panel configuration:
    - Pin Broadcast Frame Data
    - Stream Rigid body: True
    - Type: Multicast 
    - Command port: 1510
    - Data port: 1511
    - Local interface: Ip address of the network device communicating with the remote computer (i.e. 172.22.120.80)
    - Multicast Interface: Ip address of the device receiving data

