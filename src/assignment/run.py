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
upper_cabbage = np.array([73,139, 200])
lower_cabbage = np.array([63, 30, 30])

threshold_array = [(lower_lettuce, upper_lettuce), (lower_weed, upper_weed), (lower_cabbage, upper_cabbage)]

way_points = [(-4.9, 0),(4.9, 0),(4.9, 3.0),(-4.9, 3.0),(-4.9, 1.0),(4.9, 1.0), (4.9, 0.0), (-4.9, 0.0), (-4.9, -2.0), ( 4.9, -2.0), (4.9, -3.0), (-4.9, -3.0)]
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
    global path_resolution
    
    for i in range(1, len(way_points)):
        start = way_points[i-1]
        end = way_points[i]
        
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.sqrt(dx*dx+dy*dy)
        
        cnt = int(distance/path_resolution)
        
        increment_x = dx/cnt
        increment_y = dy/cnt
        
        for j in range(cnt):
            point = (start[0] + j*increment_x, start[1]+j*increment_y, i)
            global_path.append(point)

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
    
    path_segment = int(math.ceil(((point[2]-1)/4)))
    
    print(path_segment)
    set_robot_state(point[0],point[1],0)

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
    cv2.imshow("origial_rgb", rgb)
    

  
    '''  
    #detect lettuce
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV) #Transform BGR value to HSV;
    mask_after_segment = cv2.inRange(hsv, lower_lettuce, upper_lettuce, )
    cv2.imshow("mask_after_segment", mask_after_segment)

    #Dilate
    kernel2 = np.ones((3,3),np.uint8)#create convolution
    mask_after_dilate = cv2.dilate(mask_after_segment, kernel2, iterations=3) #mask after dalite 
    cv2.imshow("mask_after_dilate", mask_after_dilate)

    #Erode
    kernel = np.ones((2,2),np.uint8)#create convolution
    mask_afrer_erode = cv2.erode(mask_after_dilate, kernel, iterations=2) # mask after erode
    cv2.imshow("mask_after_erode", mask_afrer_erode)

    #find contours
    cnts = cv2.findContours(mask_afrer_erode.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.drawContours(rgb, cnts,-1,(255, 0, 0),-1)
    cv2.imshow("lecttuce", rgb)

    #now finished inspecting lecttuce
    

    #detect cabbage
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV) #Transform BGR value to HSV;
    mask_after_segment = cv2.inRange(hsv, lower_cabbage, upper_cabbage)
    cv2.imshow("mask_after_segment", mask_after_segment)

    #Dilate
    kernel2 = np.ones((3,3),np.uint8)#create convolution
    mask_after_dilate = cv2.dilate(mask_after_segment, kernel2, iterations=3) #mask after dalite 
    cv2.imshow("mask_after_dilate", mask_after_dilate)

    #Erode
    kernel = np.ones((2,2),np.uint8)#create convolution
    mask_afrer_erode = cv2.erode(mask_after_dilate, kernel, iterations=2) # mask after erode
    cv2.imshow("mask_after_erode", mask_afrer_erode)

    #find contours
    cnts = cv2.findContours(mask_afrer_erode.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.drawContours(rgb, cnts,-1,(0, 0, 255),-1)
    cv2.imshow("cabbage", rgb)
    '''
    
    lower, upper = threshold_array[path_segment]
    
    
    #detect cabbage
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV) #Transform BGR value to HSV;
    mask_after_segment = cv2.inRange(hsv, lower, upper)
    cv2.imshow("mask_after_segment", mask_after_segment)

    #Dilate
    kernel2 = np.ones((2,2),np.uint8)#create convolution
    mask_after_dilate = cv2.dilate(mask_after_segment, kernel2, iterations=3) #mask after dalite 
    cv2.imshow("mask_after_dilate", mask_after_dilate)

    #Erode
    kernel = np.ones((2,2),np.uint8)#create convolution
    mask_afrer_erode = cv2.erode(mask_after_dilate, kernel, iterations=2) # mask after erode
    cv2.imshow("mask_after_erode", mask_afrer_erode)

    #find contours
    cnts = cv2.findContours(mask_afrer_erode.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.drawContours(rgb, cnts,-1,(0, 255, 0),-1)
    cv2.imshow("weed", rgb)
    
    '''
    #Dilate first
    kernel2 = np.ones((3,3),np.uint8)#create convolution
    mask_after_dilate = cv2.dilate(mask_after_segment, kernel2, iterations=2) #mask after dalite 
    #Erode
    kernel = np.ones((2,2),np.uint8)#create convolution
    mask_after_erode = cv2.erode(mask_after_dilate, kernel, iterations=3) # mask after erode
    # cv2.imshow("mask_after_erode", mask_after_erode)#blank part is crop
    #cv2.imshow("lettuce", mask_after_erode)
    '''    

    '''
    #mark lettuce blue
    blue = np.zeros((w/2, h/2, 3), np.uint8)
    blue[:] = (255, 0, 0)
    mask_ground = cv2.bitwise_not(mask_after_erode)
    lettuce = cv2.bitwise_and(blue, blue, mask = mask_ground)
    cv2.imshow("lettuce", lettuce) #blue part is lecttuce
    '''
    

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
    
    #print("image:")
    
    
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
