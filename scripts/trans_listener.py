# coding=utf-8
import rospy
import tf
import geometry_msgs.msg


if __name__ == "__main__":
    # 初始化节点并指定名称
    rospy.init_node("trans_listener")
    # 创建一个tf接收对象，后续调用该对象的成员函数进行接收
    listener = tf.TransformListener()
    # 阻塞直到frame相通
    listener.waitForTransform("body", "cam", rospy.Time(), rospy.Duration(4.0))

    # 新建测试点用于进行测试
    point_in_rgbd = geometry_msgs.msg.PointStamped()
    point_in_rgbd.header.frame_id = "rgbd"
    point_in_rgbd.header.stamp = rospy.Time()
    point_in_rgbd.point.x = 0.4
    point_in_rgbd.point.y = 0.5
    point_in_rgbd.point.z = 0.3

    # 构造Rate对象使循环按指定频率运行
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            # 测试一，利用lookupTransform接收,监听对应的tf,返回平移和旋转
            # ros::Time(0)表示最近的一帧坐标变换，不能写成ros::Time::now()
            (translation, rotation) = listener.lookupTransform(
                "laser", "body", rospy.Time(0))
            print "translation body->laser:", translation, "\n"

            # 测试二，利用transformPoint应用变换
            # 将rgbd下的点转换到cam下
            tf_p_cam = listener.transformPoint("cam", point_in_rgbd)
            rospy.loginfo("trans1:rgbd_point:(%f,%f,%f) -> cam_point(%f,%f,%f)",
                          point_in_rgbd.point.x, point_in_rgbd.point.y, point_in_rgbd.point.z,
                          tf_p_cam.point.x, tf_p_cam.point.y, tf_p_cam.point.z)

            # 再将刚刚转换的点转回rgbd下，以此验证是否转换正确闭合
            tf_p_rgbd = listener.transformPoint("rgbd", tf_p_cam)
            rospy.loginfo("trans2:cam_point:(%f,%f,%f) -> rgbd_point(%f,%f,%f)\n",
                          tf_p_cam.point.x, tf_p_cam.point.y, tf_p_cam.point.z,
                          tf_p_rgbd.point.x, tf_p_rgbd.point.y, tf_p_rgbd.point.z)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
