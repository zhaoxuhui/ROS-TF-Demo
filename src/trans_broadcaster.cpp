#include<ros/ros.h>
#include<tf/tf.h>   // 引入ROS的TF框架
#include<tf/transform_broadcaster.h>    // 用于引入广播者相关API



int main(int argc, char *argv[])
{
    // 初始化并指定节点名称
    ros::init(argc, argv, "trans_broadcaster");
    // 新建句柄
    ros::NodeHandle nh;
    // 新建广播对象，后续调用它的成员函数实现广播
    tf::TransformBroadcaster broadcaster;

    // 事先定义好各坐标系之间的变换关系
    tf::Transform tf_body_plat;
    // setOrigin用于设置变换的平移部分，只支持传入向量形式数据
    tf_body_plat.setOrigin(tf::Vector3(0.3,0.6,0.1));
    // setRotation用于设置变换的旋转部分，只支持传入四元数形式数据
    tf_body_plat.setRotation(tf::createIdentityQuaternion());

    tf::Transform tf_body_rgbd;
    tf_body_rgbd.setOrigin(tf::Vector3(0.4,0.6,0.2));
    tf_body_rgbd.setRotation(tf::createIdentityQuaternion());

    tf::Transform tf_body_laser;
    tf_body_laser.setOrigin(tf::Vector3(0.8,-0.2,0.3));
    tf_body_laser.setRotation(tf::createIdentityQuaternion());

    tf::Transform tf_plat_cam;
    tf_plat_cam.setOrigin(tf::Vector3(0.1,0.3,-0.2));
    tf_plat_cam.setRotation(tf::createIdentityQuaternion());

    // 构造Rate对象使循环按指定频率运行
    ros::Rate rate(1);
    while (nh.ok())
    {
        // broadcaster发送的是StampedTransform类型对象，因此需要对Transform对象转换
        // StampedTransform的构造函数四个参数分别是：
        // 第一个参数是构造好的Transform数据
        // 第二个参数是这个Transform所对应的时间
        // 第三个参数是这个Transform的源坐标系名称(from orign)
        // 第四个参数是这个Transform的目标坐标系名称(to target)
        tf::StampedTransform stf_body_rgbd = tf::StampedTransform(tf_body_rgbd,ros::Time::now(),"body","rgbd");
        tf::StampedTransform stf_body_plat = tf::StampedTransform(tf_body_plat,ros::Time::now(),"body","plat");
        tf::StampedTransform stf_body_laser = tf::StampedTransform(tf_body_laser,ros::Time::now(),"body","laser");
        tf::StampedTransform stf_plat_cam = tf::StampedTransform(tf_plat_cam,ros::Time::now(),"plat","cam");
        
        // 调用成员函数发送广播
        broadcaster.sendTransform(stf_body_rgbd);
        broadcaster.sendTransform(stf_body_plat);
        broadcaster.sendTransform(stf_body_laser);
        broadcaster.sendTransform(stf_plat_cam);
        
        // 输出部分提示信息
        std::cout<<"broadcasting transform......"<<std::endl;
        // 休眠等待
        rate.sleep();
    }
    return 0;
}