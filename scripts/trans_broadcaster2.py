# coding=utf-8
import rospy
import tf
import geometry_msgs.msg
import tf2_ros.transform_broadcaster

if __name__ == "__main__":
    # 初始化并指定节点名称
    rospy.init_node("trans_broadcaster")
    # 新建广播对象，后续调用它的成员函数实现广播
    broadcaster = tf.TransformBroadcaster()
    
    # 相比于第一种方法，先构建好Message然后再广播，这样写起来好看一些
    tf_body_plat = geometry_msgs.msg.TransformStamped()
    tf_body_plat.header.frame_id = "body"
    tf_body_plat.header.stamp = rospy.Time(0)
    tf_body_plat.child_frame_id = "plat"
    tf_body_plat.transform.translation.x = 0.3
    tf_body_plat.transform.translation.y = 0.6
    tf_body_plat.transform.translation.z = 0.1
    tf_body_plat.transform.rotation.w = 1
    tf_body_plat.transform.rotation.x = 0
    tf_body_plat.transform.rotation.y = 0
    tf_body_plat.transform.rotation.z = 0

    tf_body_rgbd = geometry_msgs.msg.TransformStamped()
    tf_body_rgbd.header.frame_id = "body"
    tf_body_rgbd.header.stamp = rospy.Time(0)
    tf_body_rgbd.child_frame_id = "rgbd"
    tf_body_rgbd.transform.translation.x = 0.4
    tf_body_rgbd.transform.translation.y = 0.6
    tf_body_rgbd.transform.translation.z = 0.2
    tf_body_rgbd.transform.rotation.w = 1
    tf_body_rgbd.transform.rotation.x = 0
    tf_body_rgbd.transform.rotation.y = 0
    tf_body_rgbd.transform.rotation.z = 0

    tf_body_laser = geometry_msgs.msg.TransformStamped()
    tf_body_laser.header.frame_id = "body"
    tf_body_laser.header.stamp = rospy.Time(0)
    tf_body_laser.child_frame_id = "laser"
    tf_body_laser.transform.translation.x = 0.8
    tf_body_laser.transform.translation.y = -0.2
    tf_body_laser.transform.translation.z = 0.3
    tf_body_laser.transform.rotation.w = 1
    tf_body_laser.transform.rotation.x = 0
    tf_body_laser.transform.rotation.y = 0
    tf_body_laser.transform.rotation.z = 0

    tf_plat_cam = geometry_msgs.msg.TransformStamped()
    tf_plat_cam.header.frame_id = "plat"
    tf_plat_cam.header.stamp = rospy.Time(0)
    tf_plat_cam.child_frame_id = "cam"
    tf_plat_cam.transform.translation.x = 0.1
    tf_plat_cam.transform.translation.y = 0.3
    tf_plat_cam.transform.translation.z = -0.2
    tf_plat_cam.transform.rotation.w = 1
    tf_plat_cam.transform.rotation.x = 0
    tf_plat_cam.transform.rotation.y = 0
    tf_plat_cam.transform.rotation.z = 0


    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # 调用成员函数发送广播
        broadcaster.sendTransformMessage(tf_body_plat)
        broadcaster.sendTransformMessage(tf_body_laser)
        broadcaster.sendTransformMessage(tf_body_rgbd)
        broadcaster.sendTransformMessage(tf_plat_cam)
        
        print "broadcasting transform..."
        rate.sleep()
