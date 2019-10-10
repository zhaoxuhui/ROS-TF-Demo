# coding=utf-8
import rospy
import tf


if __name__ == "__main__":
    # 初始化并指定节点名称
    rospy.init_node("trans_broadcaster")
    # 新建广播对象，后续调用它的成员函数实现广播
    broadcaster = tf.TransformBroadcaster()
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # 在Python中向量、点、四元数、矩阵都表示成类似数组形式，Tuple、List、Numpy Array通用
        # 调用成员函数发送广播
        # 第一个参数是以tuple表示的平移分量
        # 第二个参数是以tuple表示的旋转分量，四元数顺序是(x, y, z, w)
        # 第三个参数是变换对应的时间
        # 第四个参数是变换的目标坐标系(to target)
        # 第五个参数是变换的源坐标系(from origin)
        broadcaster.sendTransform((0.4,0.6,0.2),(0,0,0,1),rospy.Time.now(),"rgbd","body")
        broadcaster.sendTransform((0.3,0.6,0.1),(0,0,0,1),rospy.Time.now(),"plat","body")
        broadcaster.sendTransform((0.8,-0.2,0.3),(0,0,0,1),rospy.Time.now(),"laser","body")
        broadcaster.sendTransform((0.1,0.3,-0.2),(0,0,0,1),rospy.Time.now(),"cam","plat")
        
        print "broadcasting transform..."
        rate.sleep()
