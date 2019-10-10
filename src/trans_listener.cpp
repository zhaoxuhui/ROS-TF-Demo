#include<ros/ros.h>
#include<tf/tf.h>   // 引入ROS的TF框架
#include<tf/transform_listener.h>   // 用于引入接收者相关API
#include<geometry_msgs/PointStamped.h>  // 代码中用到ROS中定义好的一些几何类型，如PointStamped

int main(int argc, char *argv[])
{
    // 初始化节点并指定名称
    ros::init(argc, argv, "trans_listener");
    // 新建句柄
    ros::NodeHandle nh;
    // 创建一个tf接收对象，后续调用该对象的成员函数进行接收
    tf::TransformListener listener;

    // 阻塞等待，直到指定的某两个frame连通，注意参数
    // 第一个参数是变换的目标坐标系(to)
    // 第二个参数是变换的源坐标系(from)
    // 第三个参数指定那一时刻的变换，如果写0则表示当前时刻
    // 第四个参数是阻塞时间长度
    listener.waitForTransform("body","cam",ros::Time(0),ros::Duration(4.0));

    // 新建测试点用于进行测试
    geometry_msgs::PointStamped point_in_rgbd;
    point_in_rgbd.header.frame_id = "rgbd";
    point_in_rgbd.header.stamp = ros::Time();
    point_in_rgbd.point.x = 0.4;
    point_in_rgbd.point.y = 0.5;
    point_in_rgbd.point.z = 0.3;

    // 构造Rate对象使循环按指定频率运行
    ros::Rate rate(1);
    while (nh.ok())
    {
        // 在正常运行的情况下利用try-catch语句尝试接收
        try
        {
            // 测试一，利用lookupTransform接收
            // 新建标记变换对象用于接收收到的变换
            tf::StampedTransform trans_body_laser;

            // 根据坐标名字查找变换，若找到赋值给传入的参数
            // 第一个参数是变换的目标坐标系(to)
            // 第二个参数是变换的源坐标系(from)
            // 第三个参数指定那一时刻的变换，如果写0则表示当前时刻。ros::Time(0)表示最近的一帧坐标变换，不能写成ros::Time::now()
            // 第四个参数表示用于接收变换数据的对象
            listener.lookupTransform("laser","body",ros::Time(0),trans_body_laser);

            // 获取变换的平移部分
            std::cout<<"translation body->laser：x="<<
            trans_body_laser.getOrigin().x()<<
            ",y="<<trans_body_laser.getOrigin().y()<<
            ",z="<<trans_body_laser.getOrigin().z()<<std::endl<<std::endl;
        
            // 测试二，利用transformPoint应用变换
            // 将rgbd下的点转换到cam下
            geometry_msgs::PointStamped tf_p_cam;
            listener.transformPoint("cam",point_in_rgbd,tf_p_cam);
            ROS_INFO("trans1:rgbd_point:(%f,%f,%f) -> cam_point(%f,%f,%f)",
            point_in_rgbd.point.x,point_in_rgbd.point.y,point_in_rgbd.point.z,
            tf_p_cam.point.x,tf_p_cam.point.y,tf_p_cam.point.z);
            
            // 再将刚刚转换的点转回rgbd下，以此验证是否转换正确闭合
            geometry_msgs::PointStamped tf_p_rgbd;
            listener.transformPoint("rgbd",tf_p_cam,tf_p_rgbd);
            ROS_INFO("trans2:cam_point:(%f,%f,%f) -> rgbd_point(%f,%f,%f)\n",
            tf_p_cam.point.x,tf_p_cam.point.y,tf_p_cam.point.z,
            tf_p_rgbd.point.x,tf_p_rgbd.point.y,tf_p_rgbd.point.z);
        }
        catch (tf::TransformException &ex)
        {
            // 如果失败输出错误信息
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
    return 0;
}