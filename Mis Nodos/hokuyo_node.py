#!/usr/bin/env python
############################################
#######     Hokuyo Node By Edd-1      ######
############################################
import rospy
import serial
import time
from sensor_msgs.msg import LaserScan

def get_data():
        start_data_collection=False
        data = []  #Array that keeps values ASCCI 
        com = "MD0044072500001\n"  #Command requests values once
        ser.write(com)

        for row in ser:
            if(start_data_collection):
                if(len(row) > 17):
                    for i in range (len(row)-2): #Data Block 64 bytes 
                        data.append(row[i])
                elif(len(row) == 1):
                    break
            elif(len(row) == 1):
                start_data_collection=True
        return data


def decoding_data(data):
        aux = 0
        str_bin = ""
        data_decoding = []
        character_encoding = 3

        for i in range(len(data)):
            data_bin=bin(int(hex(ord(data[i])),16)-48)
            aux += 1 

            for i in range(8-len(data_bin)):
                str_bin+="0"
            
            for i in range(2,len(data_bin)):    
                str_bin+=str(data_bin[i])        
            
            if(aux >= character_encoding):
                data_decoding.append(float(int(str_bin,2))/1000)
                str_bin=""
                aux=0        
        
        return data_decoding

def hokuyo_data():
    msg_laser = LaserScan()

    #---------Caracteristicas del laser "Hokuyo"---------
    
    angular_resolution  = 0.352         
    msg_laser.range_min = 0.0199999         # Distancia mínima de detección de obstáculos
    msg_laser.range_max = 5.5999999         # Distancia máxima de detección de obstáculos
    msg_laser.header.frame_id = "laser"                                 # Datos en [Radianes] 
    msg_laser.angle_increment = angular_resolution / 57.29              #  0.352 / 57.29 = 0.006144178739745
    msg_laser.angle_min = (-120 + (44 * angular_resolution)) / 57.29    # -120 + (15.488) / 57.29 = -104.512 / 57.29 = -1.824262524000698
    msg_laser.angle_max = ( 120 - (44 * angular_resolution)) / 57.29    #  120 - (15.488) / 57.29 =  104.512 / 57.29 =  1.824262524000698
    return msg_laser

def hokuyo():
    print "\n__________Start Hokuyo Node by Edd_________________\n"
    print "Data leng: ", len(data)
    rospy.init_node('hokuyo_node', anonymous=True)
    pub_laser_rages = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rate = rospy.Rate(10) 
    msg_laser = LaserScan()
    msg_laser = hokuyo_data()

    while not rospy.is_shutdown():   
        msg_laser.header.stamp=rospy.Time.now()
        msg_laser.ranges=decoding_data(get_data())
        pub_laser_rages.publish(msg_laser)
        rate.sleep()

if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyACM0', 19200)
        time.sleep(1)      

    except:
        print "Hokuyo has not been found"

    hokuyo()