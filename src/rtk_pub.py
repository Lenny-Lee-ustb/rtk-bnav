#!/usr/bin/env python3

# a script for GNSS-INS's INSPVAXA header. Convert to odometry format and pub. An example is shown following:
# "#INSPVAXA,COM3,0,93.9,FINESTEERING,2179,25487.000,00000000,03e2,757;
#  INS_ALIGNMENT_COMPLETE,INS_RTKFIXED,
# 22.60481305778,113.99030218678,44.5478,-3.3822, #lon lat height undulation
# 0.4616,0.2494,0.0314, #north east up -velocity
# 97.733607103,32.939981889,133.649272773, #roll pitch yaw
# 0.0254,0.0258,0.0342, #var-lon lat height
# 0.0108,0.0130,0.0105, #var-north east up -velocity
# 0.0400,0.0476,1.1325, #var-roll pitch yaw
# 00000000,0*93c5919d"

# tihs node is involved TF, but no gps->map. This(gps->map) transform was dealed in odom->base_footprint. 
# The code record the initial state when RTK spawn. Then the node minus the initial state. 

from math import pi, cos, sin
from turtle import position
import rospy
import utm
import tf2_ros
from nmea_msgs.msg import Sentence
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion, TransformStamped, Point
from tf.transformations import quaternion_from_euler

_angle2rad = pi/180.0
_rad2angle = 180.0/pi

class rtk_publisher:
    def __init__(self, tf_broadcast):
        self.is_Init = False
        self.initPose = Pose()
        self.initRoll = 0
        self.initPitch = 0
        self.initYaw = 0
        self.tf_broadcast = tf_broadcast
        self.datahead = None
        
    def callback(self,data):
        self.datahead = data.header
        datastr = data.sentence
        str = datastr.split(',')
        pose_rtk = Pose()

        if str[0] == '#INSPVAXA':
            # add orientation
            roll = float(str[18]) * _angle2rad
            pitch = float(str[19]) * _angle2rad
            # Have a pi/2 here, due to the Bnav-rtk count y-axis as front             
            yaw = pi/2.0 - float(str[20]) * _angle2rad -self.initYaw
            quat = quaternion_from_euler(roll, pitch, yaw, axes='rxyz') # r p y
            # quat = quaternion_from_euler(roll, pitch, yaw) # r p y
            pose_rtk.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3]) 

            # add position
            pos_lat = float(str[11])
            pos_lon = float(str[12])
            # transpose to X,Y
            pos = utm.from_latlon(pos_lat, pos_lon)
            X = pos[0] - self.initPose.position.x # East X
            Y = pos[1] - self.initPose.position.y # North Y
            # transfer is pi/2-init, same reason as yaw angle transfer
            pose_rtk.position = self.rotatePt(X,Y,pi/2.0-self.initYaw)
            # pose_rtk.position = self.rotatePt(X,Y,0)
            # pose_rtk.position.z = float(str[13]) # Height
            pose_rtk.position.z = 0
            
            Var_Lat = float(str[21])
            Var_Long = float(str[22])
            Var_Height = float(str[23])
            Var_Roll = float(str[27]) * _angle2rad
            Var_Pitch = float(str[28]) * _angle2rad
            Var_Yaw = float(str[29]) * _angle2rad
            
            # covariance of pose
            pose_cov_rtk = [Var_Lat*Var_Lat, 0, 0, 0, 0, 0,
                            0, Var_Long*Var_Long, 0, 0, 0, 0,
                            0, 0, Var_Height*Var_Height, 0, 0, 0,
                            0, 0, 0, Var_Roll*Var_Roll, 0, 0,
                            0, 0, 0, 0, Var_Pitch*Var_Pitch, 0, 
                            0, 0, 0, 0, 0, Var_Yaw*Var_Yaw] # 6x6 x, y, z, roll, pitch, yaw
            
            twist_rtk = Twist() 
            # add linear velocity
            twist_rtk.linear.x = float(str[15]) # north velocity
            twist_rtk.linear.y = float(str[16]) # east velocity
            twist_rtk.linear.z = float(str[17]) # up velocity

            # variance of linear velocity
            Var_vx = float(str[24])
            Var_vy = float(str[25])
            Var_vz = float(str[26])
            
            # covariance of twist
            twist_cov_rtk =[Var_vx*Var_vx, 0, 0, 0, 0, 0,
                            0, Var_vy*Var_vy, 0, 0, 0, 0,
                            0, 0, Var_vz*Var_vz, 0, 0, 0,
                            0, 0, 0, 999, 0, 0,
                            0, 0, 0, 0, 999, 0, 
                            0, 0, 0, 0, 0, 999] 
                            # 6x6 vx(not sure), vy(not sure), vz, v_roll, v_pitch, v_yaw

            if str[10] == ('INS_RTKFIXED' or 'INS_PSRDI') and self.is_Init:
            # if str[9] == ('757;INS_SOLUTION_GOOD') and self.is_Init:
                # pub msgs !
                self.pub_data(pose_rtk, pose_cov_rtk,twist_rtk, twist_cov_rtk)
                self.br_tf(pose_rtk, self.datahead.stamp, 'odom', 'base_footprint')
                # rospy.loginfo("%s %s" %(str[9], str[10]))
                # print('yaw_ori (%.2f)'%(float(str[20])))
                # print('yaw (%.2f)'%(yaw*_rad2angle))
                # print('X=%.2f, Y=%.2f'%(X,Y))
                # print('x=%.2f, y=%.2f'%(pose_rtk.position.x,pose_rtk.position.y))
            elif str[10] == 'INS_RTKFIXED' and not self.is_Init:
                self.initPose.position.x = X
                self.initPose.position.y = Y
                self.initYaw = (yaw + 2*pi)%(2*pi)
                self.is_Init = True
                print('Init pos (%.2f, %.2f)'%(self.initPose.position.x, self.initPose.position.y))
                print('Init yaw (%.2f)'%(self.initYaw*_rad2angle))
            else:
                rospy.logwarn("No RTK Signal %s %s !!" %(str[9], str[10]))

    def rotatePt(self, x, y, yaw):
        rotPt = Point()
        rotPt.x = cos(yaw)*x - sin(yaw)*y
        rotPt.y = sin(yaw)*x + cos(yaw)*y
        return  rotPt

    def pub_data(self,pose, pose_cov, twist, twist_cov):
        pub_rtk = Odometry()
        # put pos into msgs
        pub_rtk.header.stamp = self.datahead.stamp
        pub_rtk.header.frame_id = 'odom'
        pub_rtk.child_frame_id = 'base_footprint'
        pub_rtk.pose.pose.position = pose.position
        pub_rtk.pose.pose.orientation = pose.orientation
        pub_rtk.pose.covariance = pose_cov
        pub_rtk.twist.twist = twist
        pub_rtk.twist.covariance = twist_cov
        # Pub it!
        pub.publish(pub_rtk)
        

    def br_tf(self, pose, stamp, frame_id, child_frame_id):
        # init a tranform to tf2
        br_rtk = tf2_ros.TransformBroadcaster()
        rtk_tf_stamped = TransformStamped()
        rtk_tf_stamped.header.stamp = stamp
        rtk_tf_stamped.header.frame_id = frame_id
        rtk_tf_stamped.child_frame_id = child_frame_id
        rtk_tf_stamped.transform.translation = pose.position
        rtk_tf_stamped.transform.rotation = pose.orientation
        if self.tf_broadcast : # check the param
            br_rtk.sendTransform(rtk_tf_stamped)
    

if __name__ == '__main__':
    try:
        rospy.init_node('rtk_publisher', anonymous=True)
        # tf_broadcast = rospy.get_param('tf_broadcast')
        tf_broadcast = True
        rtk = rtk_publisher(tf_broadcast)
        rospy.Subscriber('nmea_sentence', Sentence, rtk.callback)
        pub = rospy.Publisher('GPS_odom', Odometry, queue_size=5, latch=True)
        rospy.loginfo("start RTK node!")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start rtk node.')
