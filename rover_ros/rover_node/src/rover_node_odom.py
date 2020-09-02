#!/usr/bin/env python
import rospy
import spidev
import serial
import tf
from math import cos
from math import sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rover_node.msg import batteryState
from  recursive_bayesian_filter_ros.msg import OdomMeasurementAckermann
from nav_msgs.msg import Odometry
import struct
from time import sleep
global state
odom=Odometry()
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
    pub1 = rospy.Publisher('rover_odom', Odometry, queue_size=10)
    pub2 = rospy.Publisher('rover_batteryVolt', batteryState, queue_size=10)
    pub3 = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.Subscriber("cmd_vel", Twist, callback)
    size=58         #Buffer size to receive
    if com == 'SPI':
        rospy.loginfo('[rover_node] Node started');
        rospy.loginfo("[rover_node] SPI protocol is used to communicate with the rove MCU")
    elif com == 'UART':
        rospy.loginfo('[rover_node] Node started');
        rospy.loginfo("[rover_node] UART protocol is used to communicate with the rove MCU")
        receivedData=size*[0];
    rate = rospy.Rate(50) 
    br = tf.TransformBroadcaster()
    local_com_id=-1
    pass_odom=0
    count=0
    while not rospy.is_shutdown():
                if com == 'SPI':
                    ### SPI transaction
                    #print "tosend:", tosend
                    xfer_send=tosend[:]+(size-8)*[0] #bytes to send   
                    data_read=spi.xfer(xfer_send,spi_rate) #Full-duplex communication with spi_rate rate
                    if local_com_id == -1: 
                        timestamp=rospy.Time.now();
                    else:
                        local_com_id+=1
                        timestamp=timestamp+rospy.Duration(0.02) 
                        #timestamp=rospy.Time.now();
                    ###Pseudo synchronization:
                    Buffer=[struct.pack("B",x) for x in data_read[size-2:size]]
                    Buffer=''.join(Buffer)
                    [com_id]=struct.unpack("H",Buffer)
                    if local_com_id==65531 and com_id==1:
                            local_com_id=1 
                    elif local_com_id==65530 and com_id==1:
                        local_com_id=0
#                    print "lid:", local_com_id, "// id:", com_id                  
                    pass_odom=0
                    if local_com_id == -1 and com_id!=0:
                        local_com_id=com_id
                        rospy.logdebug("Start synchronization")
                    elif com_id!=0 and com_id >local_com_id:
                        local_com_id=com_id
                        count+=1
                        timestamp=timestamp+rospy.Duration(0.02) 
                        rospy.logdebug("Missed stamp")
                    elif com_id<local_com_id:
                        pass_odom=1
                        local_com_id=com_id
                        count-=1
                        timestamp=timestamp-rospy.Duration(0.02) 
                        rospy.logdebug("Duplicated stamp")
                    elif com_id == local_com_id:
                        rospy.logdebug("Well synchronized")
 #                   print "rospy:", rospy.Time.now(), "timestamp:", timestamp, "count:", count                   
                    ### Unpacking data
                    data_read=data_read[:size-2]
                    Buffer=[struct.pack("B",x) for x in data_read]
                    Buffer=''.join(Buffer)
                    receivedData=struct.unpack("14f",Buffer)
                elif com == 'UART':
                    nbytes=int(ser.inWaiting())
                    if(nbytes/size>=1 and nbytes%size==0 ):
                        data_read=ser.read(nbytes)
                        data_read=data_read[nbytes-size:nbytes]
                        receivedData=struct.unpack('14f',data_read)
                if pass_odom==0:
                    ### Topics publication
                    servoPose=receivedData[1]
                    odom.header.stamp=timestamp; 
                    odom.header.frame_id = "odom";
                    odom.child_frame_id = "base_link";
                    odom.pose.pose.position.x = receivedData[2];
                    odom.pose.pose.position.y = receivedData[3];
                    odom.pose.pose.position.z = 0;
                    odom.pose.pose.orientation.w = cos(receivedData[4]/2) ;
                    odom.pose.pose.orientation.x = 0;
                    odom.pose.pose.orientation.y = 0;
                    odom.pose.pose.orientation.z = sin(receivedData[4]/2);

                    odom.pose.covariance[0] = receivedData[5];    
                    odom.pose.covariance[1] = receivedData[6];   
                    odom.pose.covariance[2] = 0;                
                    odom.pose.covariance[3] = 0;                      
                    odom.pose.covariance[4] = 0;                      
                    odom.pose.covariance[5] = receivedData[7];    

                    odom.pose.covariance[6] =  receivedData[8];    
                    odom.pose.covariance[7] =  receivedData[9];   
                    odom.pose.covariance[8] =  0;                      
                    odom.pose.covariance[9] =  0;                     
                    odom.pose.covariance[10] = 0;                    
                    odom.pose.covariance[11] = receivedData[10];    

                    odom.pose.covariance[12] = 0;
                    odom.pose.covariance[13] = 0;
                    odom.pose.covariance[14] = 0;
                    odom.pose.covariance[15] = 0;
                    odom.pose.covariance[16] = 0;
                    odom.pose.covariance[17] = 0;

                    odom.pose.covariance[18] = 0;
                    odom.pose.covariance[19] = 0;
                    odom.pose.covariance[20] = 0;
                    odom.pose.covariance[21] = 0;
                    odom.pose.covariance[22] = 0;
                    odom.pose.covariance[23] = 0;

                    odom.pose.covariance[24] = 0;
                    odom.pose.covariance[25] = 0;
                    odom.pose.covariance[26] = 0;
                    odom.pose.covariance[27] = 0;
                    odom.pose.covariance[28] = 0;
                    odom.pose.covariance[29] = 0;

                    odom.pose.covariance[30] = receivedData[11];     
                    odom.pose.covariance[31] = receivedData[12];    
                    odom.pose.covariance[32] = 0;                        
                    odom.pose.covariance[33] = 0;                       
                    odom.pose.covariance[34] = 0;                       
                    odom.pose.covariance[35] = receivedData[13];     

                    pub1.publish(odom)

                    battery.batteryVoltage=receivedData[0]
                    battery.header.stamp=timestamp;
                    pub2.publish(battery)
                    
                    steeringState.header.stamp=timestamp;
                    steeringState.name=['front_left_steering_joint', 'front_right_steering_joint'];
                    steeringState.position=2*[receivedData[1]];
                    steeringState.header.seq+=1;
                    pub3.publish(steeringState)

                    wheelsState.header.stamp=timestamp;
                    wheelsState.header.seq+=1;
                    wheelsState.name=['rear_left_wheel_joint','rear_right_wheel_joint','front_left_wheel_joint','front_right_wheel_joint']
                    wheelsState.position=4*[0];
                    pub3.publish(wheelsState)
                    
                    ### tf publish
                    br.sendTransform((odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),
                         (odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w),
                         timestamp,
                         "base_link",
                         "odom")
                rate.sleep()


if __name__ == '__main__':
    try:
        rover_node()

    except rospy.ROSInterruptException:
        pass

