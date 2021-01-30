#!env /usr/bin python
import rospy
import cv2
#i add
import cv2 as cv
import math
import numpy as np
#from detection import detection
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState
import tf.transformations as tft
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench

flag = 1
cmd = Twist()

line_run_cnt = 0
line_run_dir = 1

set_model_state_client = None

#define threshold of 3 types of plants
#lettuce
upper_lettuce = np.array( [80, 137, 60])
lower_lettuce = np.array([66, 125, 33])

#weed
upper_weed = np.array( [60,80, 110])
lower_weed = np.array([40, 60, 50])

#cabbage
upper_cabbage = np.array([85,175, 80])
lower_cabbage = np.array([70, 165, 55])

threshold_array = [(lower_lettuce, upper_lettuce), (lower_weed, upper_weed), (lower_cabbage, upper_cabbage)]

way_points = [(-5.5, 0),(5.5, 0),(5.5, 4.0),(-5.5, 4.0),(-5.5, -3.0),(5.5, -3), (5.5, 3.0), (-5.5, 3.0), (-5.5, -2.0), ( 5.5, -2.0), (5.5, 1.0), (-5.5, 1.0)]
global_path = []
path_resolution = 0.1
path_segment = 0

def line_run():
    global cmd, line_run_cnt,line_run_dir
    if(line_run_cnt > 100):
        line_run_dir = -1
    elif(line_run_cnt < 0):
        line_run_dir = 1

    line_run_cnt += line_run_dir

    #cmd.linear.x = 0.5 * line_run_dir
    cmd.linear.y = 0.5 * line_run_dir
    cmd.angular.z = 0.0

def generate_global_path(way_points):
    global path_resolution, global_path
    R = 0.1
    
    S = None
    A = None
    B = None
    yaw_AB = None
    for i in range(1, len(way_points)):
        if(i == 1):
            A = way_points[i-1]
            B = way_points[i]
            dx = B[0] - A[0]
            dy = B[1] - A[1]
            distance = math.sqrt(dx*dx+dy*dy)
            yaw_AB = math.atan2(dy, dx)
            cnt = int(math.ceil(distance/path_resolution))
            inc_x = dx/cnt
            inc_y = dy/cnt
        
            inc_l = math.sqrt(inc_x*inc_x + inc_y*inc_y)
            cnt = int(math.ceil((distance-R)/inc_l))
            
            # line A-S
            for j in range(cnt):
                point = (A[0] + j*inc_x, A[1]+j*inc_y, yaw_AB)
                global_path.append(point)
            
            S = (A[0] + cnt*inc_x, A[1]+cnt*inc_y)
            
        else:
            C = way_points[i]
            
            dx = C[0] - B[0]
            dy = C[1] - B[1]
#            print(B, C)
            distance = math.sqrt(dx*dx+dy*dy)
            yaw_BC = math.atan2(dy, dx)
            cnt_BC = int(math.ceil(distance/path_resolution))
            inc_x = dx/cnt_BC
            inc_y = dy/cnt_BC
        
            inc_l = math.sqrt(inc_x*inc_x + inc_y*inc_y)
            cnt_BE = int(math.ceil(R/inc_l))
            cnt_EC = cnt_BC - cnt_BE
            cnt_ES_next = cnt_BC - cnt_BE*2
            E = (B[0] + cnt_BE*inc_x, B[1]+cnt_BE*inc_y)
            
            #calculate C in (A->B) coordinate
            local_y = -(C[0]-B[0])*math.sin(yaw_AB) + (C[1]-B[1])*math.cos(yaw_AB)
            
            if(local_y >= 0):
                sign = 1
            else:
                sign = -1
#            print(sign, yaw_AB)
            #the center of turn in global coordinate
#            O = (sign* R*math.sin(yaw_AB) + S[0], sign* R*math.cos(yaw_AB) + S[1])
            O = (-sign* R*math.sin(yaw_AB) + S[0], sign* R*math.cos(yaw_AB) + S[1])
            
#            global_path.append((O[0],O[1],0))
            yaw_OS = math.atan2(S[1]-O[1], S[0]-O[0])
            
            arc_len = R*math.pi/2
            cnt_SE = int(math.ceil(arc_len/path_resolution))
#            print(cnt_SE)
            # arc S-E
            for j in range(cnt_SE):
                theta = sign * 1.0*j/cnt_SE * math.pi/2
#                print(theta)
                local_x = R*math.cos(theta)
                local_y = R*math.sin(theta)
                
                global_x = local_x*math.cos(yaw_OS) - local_y*math.sin(yaw_OS) + O[0]
                global_y = local_x*math.sin(yaw_OS) + local_y*math.cos(yaw_OS) + O[1]
                yaw_arc_p = yaw_OS + theta
                
                global_path.append((global_x, global_y, yaw_arc_p))
            
            if(i == len(way_points)-1):
                cnt_ES_next = cnt_EC
            
            for j in range(cnt_ES_next):
                point = (E[0] + j*inc_x, E[1]+j*inc_y, yaw_BC)
                global_path.append(point)
            
            A = B
            B = C
            yaw_AB = yaw_BC
            S = (E[0] + cnt_ES_next*inc_x, E[1]+cnt_ES_next*inc_y)
        
   

def set_robot_state(x,y,yaw):
    global set_model_state_client
    
    req = SetModelStateRequest()
    req.model_state.model_name = "thorvald_001"
    req.model_state.reference_frame = ''
    
    req.model_state.pose.position.x = x
    req.model_state.pose.position.y = y
    req.model_state.pose.position.z = 0
    
    tmpq = tft.quaternion_from_euler(0, 0, yaw)
    q = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])
    req.model_state.pose.orientation = q
    
    response = set_model_state_client.call(req)
    print(response)
    
def path_tracking(index):
    global global_path, path_segment
    
    if(index >= len(global_path)):
        return
    
    point = global_path[index]
    
    set_robot_state(point[0],point[1],point[2])

def image_callback(rgb_img, depth_img):
    global flag, cmd, upper_lettuce, upper_weed, lower_weed, upper_cabbage, lower_cabbage, threshold_array, path_segment
    
    rgb  = np.frombuffer(rgb_img.data, dtype=np.uint8).reshape(rgb_img.height, rgb_img.width, -1)
    depth = np.frombuffer(depth_img.data, dtype=np.uint16).reshape(depth_img.height, depth_img.width, -1) #convert ros image to opencv image
    
    shape = rgb.shape
    w = shape[0]
    h = shape[1]
    #print("%d x %d" %(w,h))
    
    '''
    if(flag):
        cv2.imwrite("/home/bonjour/Downloads/rgb2.png", rgb)
        flag = 0
    '''
    size = (int(h/2),int(w/2))
    rgb = cv2.resize(rgb, size)
    depth = cv2.resize(depth, size)
    #cv2.imshow("origial_rgb", rgb)
    
    lower, upper = threshold_array[path_segment]
    #segment
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV) #Transform BGR value to HSV;
    segment_weed = cv2.inRange(hsv, lower_weed, upper_weed)
    segment_lettuce = cv2.inRange(hsv, lower_lettuce, upper_lettuce)
    segment_cabbage = cv2.inRange(hsv, lower_cabbage, upper_cabbage)
    #cv2.imshow("mask_after_segment", mask_after_segment)

    #Dilate
    kernel2 = np.ones((2,2),np.uint8)#create convolution
    dilate_weed = cv2.dilate(segment_weed, kernel2, iterations=3) #mask after dalite 
    dilate_lettuce = cv2.dilate(segment_lettuce, kernel2, iterations=3)
    dilate_cabbage = cv2.dilate(segment_cabbage, kernel2, iterations=3)
    #cv2.imshow("mask_after_dilate", mask_after_dilate)

    #Erode
    kernel = np.ones((2,2),np.uint8)#create convolution
    erode_weed = cv2.erode(dilate_weed, kernel, iterations=2)
    erode_lettuce = cv2.erode(dilate_lettuce, kernel, iterations=2)
    erode_cabbage = cv2.erode(dilate_cabbage, kernel, iterations=2) # mask after erode
    #cv2.imshow("mask_after_erode", mask_afrer_erode)

    #find contours
    cnt_weed = cv2.findContours(erode_weed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts_lettuce = cv2.findContours(erode_lettuce.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts_cabbage = cv2.findContours(erode_cabbage.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    #background = np.zeros([w/2, h/2],dtype=np.uint8)
    cv2.drawContours(rgb, cnts_lettuce,-1,(0, 255, 0),-1)
    cv2.drawContours(rgb, cnts_cabbage,-1,(30, 255, 255),-1)
    cv2.drawContours(rgb, cnt_weed,-1,(0, 0, 255),-1)
    cv2.imshow("3 types of plants", rgb)

    cv2.waitKey(20)
    return #stop here to debug

   
    print(cnts)
    plant_center = None #Initialize the center of red object;
    
    c = max(cnts, key=cv2.contourArea) #find the largest largest contour
    (a, b), r = cv2.minEnclosingCircle(cnts[0]) 
    #r,g,b = cv2.split(rgb)
    #cv2.imshow("g", g)
     
    cv2.imshow("image",rgb)
    #print(x,y)#the next waypoint

    cv2.waitKey(20)
    
#    cmd.linear.x = 0.5
#    cmd.angular.z = 0.5

    #line_run()
    


def main():
    global cmd, set_model_state_client, way_points, global_path
    rospy.init_node("assignment_node")
#    cv2.namedWindow("image",0)
#    cv2.namedWindow("mask",0)
    
    server_name = "/gazebo/set_model_state"
    rospy.wait_for_service(server_name)
    set_model_state_client = rospy.ServiceProxy(server_name, SetModelState)
    generate_global_path(way_points)
    
    rate = rospy.Rate(10)
    
    depth_img_sub = message_filters.Subscriber('/thorvald_001/kinect2_sensor/sd/image_depth_rect', Image)
    rgb_img_sub   = message_filters.Subscriber('/thorvald_001/kinect2_camera/hd/image_color_rect', Image)
    cmd_pub = rospy.Publisher("/thorvald_001/turtle1/cmd_vel", Twist, queue_size = 1)
    #img_pub = rospy.Publisher("/thorvald_001/kinect2_camera/hd/image_color_rect", rgb, queue_size = 1)

    sync = message_filters.TimeSynchronizer([rgb_img_sub, depth_img_sub], 10)
    sync.registerCallback(image_callback)
    
    index = 0
    
    while not rospy.is_shutdown():
        #cmd_pub.publish(cmd)
        
        path_tracking(index)
        index += 1
        
        rate.sleep()
    
    rospy.spin()
    
if __name__ == '__main__':
    main()
    try:
        main()
    except:
        pass
