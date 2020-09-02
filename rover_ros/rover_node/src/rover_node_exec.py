#!/usr/bin/env python
import rospy
import spidev
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rover_node.msg import batteryState
from  recursive_bayesian_filter_ros.msg import OdomMeasurementAckermann
import struct
from time import sleep
global state
com=rospy.get_param('/rover_node/com','SPI')
if com == 'UART':
    serial_port=rospy.get_param('/rover_node/serial_port','/dev/ttyS0')
    serial_baud=rospy.get_param('/rover_node/serial_baud',115200)
    ser = serial.Serial (serial_port, serial_baud)    #Open port with baud rate
    state=OdomMeasurementAckermann()
elif com == 'SPI':
    spi_device=rospy.get_param('/rover_node/spi_device',0)
    spi_bus=rospy.get_param('/rover_node/spi_bus',0)
    spi_rate=rospy.get_param('/rover_node/spi_rate',1000000)
    spi = spidev.SpiDev()
    spi.open(spi_bus,spi_device)  
    spi.mode = 0b01
    #spi.no_cs=True
    #spi.cshigh=False
    state=OdomMeasurementAckermann()
    global tosend
    tosend=8*[0x00]

def callback(data):
    global tosend
    bldcSpeed=data.linear.x
    servoSpeed=-data.angular.z
    bldc=struct.pack('f',bldcSpeed)
    servo=struct.pack('f',servoSpeed)
    if com == 'UART':
        ser.write(bldc+servo)                #transmit data serially
    #    print "bldc:",bldcSpeed," Servo:",servoSpeed
    elif com == 'SPI':
        tosend=[struct.unpack("B",x)[0] for x in list(struct.pack("f",bldcSpeed))+list(struct.pack("f",servoSpeed))]
        #print tosend    

def rover_node():
    global tosend
    battery=batteryState()
    steeringState=JointState()
    wheelsState=JointState()
    rospy.init_node('rover_node', anonymous=True)
    pub1 = rospy.Publisher('rover_state',  OdomMeasurementAckermann, queue_size=10)
    pub2 = rospy.Publisher('rover_batteryVolt', batteryState, queue_size=10)
    pub3 = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.Subscriber("cmd_vel", Twist, callback)
    if com == 'SPI':
        rospy.loginfo('[rover_node] Node started');
        rospy.loginfo("[rover_node] SPI protocol is used to communicate with the rove MCU")
    elif com == 'UART':
        rospy.loginfo('[rover_node] Node started');
        rospy.loginfo("[rover_node] UART protocol is used to communicate with the rove MCU")
        size=28         #Buffer size to receive
    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
                if com == 'SPI':
                    ### SPI transaction
                    #print "tosend:", tosend
                    xfer_send=tosend[:]+20*[0]    
                    data_read=spi.xfer(xfer_send,spi_rate) #Full-duplex communication of spi_rate rate
                    #data_read=spi.xfer([33,6,70,8,5,6,56,8]+16*[0],1000000) #Full-duplex communication of 1mb/s rate
                    #print data_read

                    ### Unpacking data
                    Buffer=[struct.pack("B",x) for x in data_read]
                    Buffer=''.join(Buffer)
                    receivedData=struct.unpack("7f",Buffer)
                elif com == 'UART':
                    nbytes=int(ser.inWaiting())
                    receivedData=size*[0];
                    if(nbytes/size>=1 and nbytes%size==0 ):
                        data_read=ser.read(nbytes)
                        data_read=data_read[nbytes-size:nbytes]
                        receivedData=struct.unpack('7f',data_read)


                speed=receivedData[0]
                servoPose=receivedData[1]
                variance=receivedData[2:6]
                batteryVolt=receivedData[6]
                state.v=speed
                state.header.stamp = rospy.Time.now();
                state.header.seq+=1;
                state.phi=servoPose
                state.covariance=variance
                #print "speed:" , speed, "servo:", servoPose
                #print variance
                pub1.publish(state)
                battery.batteryVoltage=batteryVolt
                battery.header.stamp=rospy.Time.now();
                pub2.publish(battery)
                
                steeringState.header.stamp=rospy.Time.now();
                steeringState.name=['front_left_steering_joint', 'front_right_steering_joint'];
                steeringState.position=[servoPose,servoPose];
                steeringState.header.seq+=1;
                pub3.publish(steeringState)

                wheelsState.header.stamp=rospy.Time.now();
                wheelsState.header.seq+=1;
                wheelsState.name=['rear_left_wheel_joint','rear_right_wheel_joint','front_left_wheel_joint','front_right_wheel_joint']
                wheelsState.velocity=4*[speed];
                wheelsState.position=4*[0];
                pub3.publish(wheelsState)
                
                rate.sleep()

if __name__ == '__main__':
    try:
        rover_node()

    except rospy.ROSInterruptException:
        pass

