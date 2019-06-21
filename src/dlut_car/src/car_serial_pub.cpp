#include <sstream>
#include <serial/serial.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <cartographer_ros_msgs/StartTrajectory.h>
#include <cartographer_ros_msgs/FinishTrajectory.h>
#include <cartographer_ros_msgs/WriteState.h>

serial::Serial ros_ser;
ros::ServiceClient CartographerStartTrajectory;
ros::ServiceClient CartographerFinishTrajectory;
ros::ServiceClient CartographerWriteState;
union SendDataType
{
    struct
    {
        uint8_t Header[4];
        int32_t speed;
        int32_t dir;
        uint8_t ctrl;
        uint8_t mode;
        uint8_t state;
        uint8_t sum;
    }data;
    uint8_t buf[16]; 
}SendData;


using namespace std;

void TwistCallback(const geometry_msgs::Twist& twist)
{
    int speed = 0,dir = 0;
    //double angle;
    if(twist.linear.x>0.05)
        speed = 155;
    else
        speed = 0;
    dir = (int)(twist.angular.z * -1500);
    if(dir>400)
        dir = 400;
    else if(dir<-400)
        dir = -400;

    SendData.data.speed = speed;
    SendData.data.dir = dir;
    //angle = 2500.0 - twist.angular.z * 2000.0 / 180.0;
    //ROS_INFO("angle= %d",uint16_t(angle));
    //send_cmd(uint16_t(twist.linear.x),uint16_t(angle));
}

void Twist_ctrl_Callback(const geometry_msgs::Twist& twist)
{
    ROS_INFO("v= %f", twist.linear.x);
    ROS_INFO("a= %f", twist.angular.z);
}

void ComSendData(void)
{
    SendData.data.sum = 0;
    for(int i = 0;i<15;i++)
        SendData.data.sum += SendData.buf[i];
    ros_ser.write(SendData.buf,sizeof(SendData));
}

/*
        {   //结束建图
            cartographer_ros_msgs::FinishTrajectory f_srv;
            f_srv.request.trajectory_id = 0;
            if(SendData.data.mode != 1)
            {
                ROS_INFO("FinishiTrajectory Not Allow with mode=%d",SendData.data.mode);
                return;
            }
            if(CartographerFinishTrajectory.call(f_srv))
                SendData.data.mode = 2;
            else
            {
                SendData.data.state |= 0x02;
                ROS_ERROR("FinishiTrajectory Failed!");
            }
        }
        else if(RecData.data.type == 3)
        {   //保存地图
            cartographer_ros_msgs::WriteState w_srv;            
            w_srv.request.filename = "/home/zbt/save.pbstream";
            CartographerWriteState.call(w_srv);
            ROS_INFO("Save map~");
        }
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dlut_cardriver");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/cmd_vel_joy",1,TwistCallback);
    ros::Subscriber sub_cmd = n.subscribe("/cmd_vel",1,Twist_ctrl_Callback);
    CartographerStartTrajectory = n.serviceClient<cartographer_ros_msgs::StartTrajectory>("start_trajectory");
    CartographerFinishTrajectory = n.serviceClient<cartographer_ros_msgs::FinishTrajectory>("finish_trajectory");
    CartographerWriteState = n.serviceClient<cartographer_ros_msgs::WriteState>("write_state");
    ros::Rate loop(100);

    //串口初始化
    try
    {
         ros_ser.setPort("/dev/ttyUSB0"); ///dev/car
         ros_ser.setBaudrate(115200);
         serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(3);
         ros_ser.setTimeout(serial_timeout);
         ros_ser.open();
    }
    catch (serial::IOException& e)
    {
         ROS_ERROR_STREAM("Unable to open port /dev/car");
         return -1;
    }
    if(ros_ser.isOpen())
         ROS_INFO_STREAM("Serial Port /dev/car opened");
    else
         return -1;

    //初始化数据包内容
    for(int i = 0;i<sizeof(SendData);i++)
        SendData.buf[i] = 0;
    SendData.data.Header[0] = SendData.data.Header[1]
        = SendData.data.Header[2] = SendData.data.Header[3] = 0x5B;

    while (ros::ok())
    {
        ros::spinOnce();
        ComSendData();
        if(ros_ser.available())
            ros_ser.flushInput();   //暂不接受回传数据
        loop.sleep();
    }

    return 0;
}