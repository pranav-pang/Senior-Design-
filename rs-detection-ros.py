import pyrealsense2 as rs 
import numpy as np 
import cv2 
import math
import rospy
from std_msgs.msg import String 

def publish():
    pub = rospy.Publisher('viodata', String, queue_size=10)
    rospy.init_node('vio', anonymous=True)
    rate = rospy.rate(10)
    pipeline = rs.pipeline() 
    config = rs.config() 

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    w_max = 1280
    h_max = 720
    fps = 30

    config.enable_stream(rs.stream.depth, w_max, h_max, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, w_max, h_max, rs.format.bgr8, fps)

    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth scale: ", depth_scale)

    clipping_distance_meters = 1
    clipping_distance = clipping_distance_meters / depth_scale

    align_to = rs.stream.color
    align = rs.align(align_to) 

    fov_x = 86
    fov_y = 57
    last_updated_dist = 0
    last_updated_height = 0
    last_updated_depth = 0

    while not rospy.is_shutdown():
        try:
            while True:
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)

                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not aligned_depth_frame or not color_frame:
                    continue 

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
                hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

                red_lower = np.array([136, 87, 111], np.uint8)
                red_upper = np.array([180, 255, 255], np.uint8)
                red_mask = cv2.inRange(hsv_image, red_lower, red_upper)

                kernel = np.ones((5, 5), "uint8")

                red_mask = cv2.dilate(red_mask, kernel)
                res_red = cv2.bitwise_and(color_image, color_image, 
                                        mask = red_mask)
                _, contours, hierarchy = cv2.findContours(red_mask,
                                                    cv2.RETR_TREE,
                                                    cv2.CHAIN_APPROX_SIMPLE)
            
                bboxes = {}
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if(area > 300):
                        x, y, w, h = cv2.boundingRect(contour) 
                        bboxes[w * h] = {"x": x, "y": y, "w": w, "h": h}

                #color_image = cv2.circle(color_image, (480, 10), 3, (0, 255, 0), 2)

                if (len(bboxes) != 0):
                    key = max(bboxes.keys())
                    x = bboxes[key]["x"]
                    y = bboxes[key]["y"]
                    w = bboxes[key]["w"]
                    h = bboxes[key]["h"]
                    color_image = cv2.rectangle(color_image, (x, y), 
                                                (x + w, y + h), 
                                                (0, 0, 255), 2)
                    xmin_depth = y 
                    ymin_depth = x 
                    xmax_depth = xmin_depth + h 
                    ymax_depth = ymin_depth + w 
                    depth_boundingbox = depth_image[xmin_depth:xmax_depth, ymin_depth:ymax_depth].astype(float) * depth_scale
                    distance, _, _, _ = cv2.mean(depth_boundingbox) 

                    distance = distance * 100

                    if (distance == 0):
                        distance = last_updated_depth
                    else:
                        last_updated_depth = distance

                    lx = (x + w / 2) - (w_max / 2)
                    ly1 = y - (h_max / 2)
                    ly2 = (y + h) - (w_max / 2)
                    tan_angle_radian_x = math.tan((fov_x / 2) * (math.pi / 180))
                    tan_angle_radian_y = math.tan((fov_y / 2) * (math.pi / 180))

                    theta = math.atan2(lx, (w_max / 2) * (1 / tan_angle_radian_x)) * (180 / math.pi) 
                    alpha1 = math.atan2(ly1, (h_max / 2) * (1 / tan_angle_radian_y)) * (180 / math.pi) 
                    alpha2 = math.atan2(ly2, (h_max / 2) * (1 / tan_angle_radian_y)) * (180 / math.pi) 

                    idx1 = x + int(w / 2)
                    idx3 = y + h
                    if (idx3 >= h_max):
                        idx3 = h_max - 1
                    if (idx1 >= w_max):
                        idx1 = w_max - 1
                    depth_top = depth_image[y, idx1] * depth_scale * 100
                    depth_bottom = depth_image[idx3, idx1] * depth_scale * 100

                    distance_true_top = depth_top * math.cos(alpha1 * (math.pi / 180))
                    distance_true_bottom = depth_bottom * math.cos(alpha2 * (math.pi / 180))

                    distance_true = (distance_true_top + distance_true_bottom) / 2

                    height = depth_bottom * math.sin(alpha2 * (math.pi / 180)) - depth_top * math.sin(alpha1 * (math.pi / 180))

                    if depth_top == 0 or depth_bottom == 0:
                        distance_true = last_updated_dist
                        height = last_updated_height
                    else:
                        last_updated_dist = distance_true
                        last_updated_height = height

                    msg = "{:.2f} {:.2f} {:.2f}".format(distance_true, height, theta)
                    rospy.loginfo(msg)
                    pub.publish(msg)
                    rate.sleep()

                #     color_image = cv2.line(color_image, (x + w + 10, y), (x + w + 10, y + h), (0, 255, 0))
                #     color_image = cv2.line(color_image, (int(w_max / 2), y - 10), (x + int(w / 2), y - 10), (255, 0, 0))

                #     cv2.putText(color_image, "Dist: {:.2f} cm".format(distance_true), (10, 55),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                #                 (0, 255, 255))

                #     cv2.putText(color_image, "Height: {:.2f} cm".format(height), (x + w, y + int(h / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))

                #     cv2.putText(color_image, "Angle: {:.2f} deg".format(theta), (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))

                #     cv2.putText(color_image, "Depths (Top, Bot): ({:.2f}, {:.2f}) cm".format(depth_top, depth_bottom), (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255))
                #     cv2.putText(color_image, "Alpha (1, 2): ({:.2f}, {:.2f}) deg".format(alpha1, alpha2), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0))
                #     cv2.putText(color_image, "D (Top, Bot): ({:.2f}, {:.2f}) cm".format(distance_true_top, distance_true_bottom), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255))

                # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                # images = np.hstack((color_image, depth_colormap))

                # cv2.namedWindow('Aligned Image', cv2.WINDOW_NORMAL)
                # cv2.imshow('Aligned Image', images)
                # key = cv2.waitKey(4)

                # if key & 0xFF == ord('q') or key == 27:
                #     cv2.destroyAllWindows()
                #     break
        finally:
            pipeline.stop()


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass


