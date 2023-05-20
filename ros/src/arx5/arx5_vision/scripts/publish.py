#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from arx5_vision.my_handeye_calibration import MyHandeyeCalibration

rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

inverse = rospy.get_param('inverse')
filename = rospy.get_param('calibration_file')

if filename == '':
    rospy.logdebug('No path specified for the calibration file, loading from the standard location')
    filename = MyHandeyeCalibration.filename_for_namespace(rospy.get_namespace())

rospy.loginfo("Loading the calibration from file: %s", filename)
calib = MyHandeyeCalibration.from_filename(filename)

if calib.parameters.eye_on_hand:
    overriding_robot_effector_frame = rospy.get_param('robot_effector_frame')
    if overriding_robot_effector_frame != "":
        calib.transformation.header.frame_id = overriding_robot_effector_frame
else:
    overriding_robot_base_frame = rospy.get_param('robot_base_frame')
    if overriding_robot_base_frame != "":
        calib.transformation.header.frame_id = overriding_robot_base_frame
overriding_tracking_base_frame = rospy.get_param('tracking_base_frame')
if overriding_tracking_base_frame != "":
    calib.transformation.child_frame_id = overriding_tracking_base_frame

rospy.loginfo('loading calibration parameters into namespace {}'.format(
    rospy.get_namespace()))
MyHandeyeCalibration.store_to_parameter_server(calib)

orig = calib.transformation.header.frame_id  # 'robot_effector_frame' or robot_base_frame （根据手眼标定的类型）
dest = calib.transformation.child_frame_id  # tracking_base_frame（即相机所在坐标系，而不是二维码）

broadcaster = tf2_ros.StaticTransformBroadcaster()
static_transformStamped = geometry_msgs.msg.TransformStamped()
static_transformStamped.header.stamp = rospy.Time.now()
static_transformStamped.header.frame_id = orig # 基准坐标系（实际我们是想“robot_base_frame”所在坐标系）
static_transformStamped.child_frame_id = dest  # 目标坐标系（"相机"所在坐标系）
static_transformStamped.transform = calib.transformation.transform  # 变换关系
# 本质上发布仅仅是手眼标定的TF数据（即相机-末端执行机构的变换）
broadcaster.sendTransform(static_transformStamped)
rospy.spin()
