#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/Pose.h"
#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/String.h>
#include <vector>
#include <iostream>
#include <string>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#define _USE_MATH_DEFINES 
#include <cmath>
#include <thread>



using namespace std;

struct Quaternion 
{
    float w, x, y, z;
};
struct EulerAngles 
{
    float roll, pitch, yaw;
};

struct EulerPose
{   
    struct Position
    {
        float x, y, z;
    };
    Position position;
    EulerAngles eular;
};



bool beGrasped = false;



// 储存原始变换信息
EulerPose tf_ori;
EulerPose link_ori;




//欧拉角转化四元数
Quaternion ToQuaternion(EulerAngles Eular)
{
    float cy = cos(Eular.yaw * 0.5);
    float sy = sin(Eular.yaw * 0.5);
    float cp = cos(Eular.pitch * 0.5);
    float sp = sin(Eular.pitch * 0.5);
    float cr = cos(Eular.roll * 0.5);
    float sr = sin(Eular.roll * 0.5);
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}
//四元数转化为欧拉角
EulerAngles ToEulerAngles(Quaternion q) 
{
    EulerAngles angles;
 
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);
    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}


void clear_tf_ori()
{
    tf_ori.position.x = 0;
    tf_ori.position.y = 0;
    tf_ori.position.z = 0;
    tf_ori.eular.pitch = 0;
    tf_ori.eular.roll = 0;
    tf_ori.eular.yaw = 0;
}

// 设置原始坐标变换
void set_tri_ori(geometry_msgs::Pose model, geometry_msgs::Pose link)
{
    Quaternion tempQuaternion;
    tempQuaternion.w = model.orientation.w;
    tempQuaternion.x = model.orientation.x;
    tempQuaternion.y = model.orientation.y;
    tempQuaternion.z = model.orientation.z;
    EulerAngles model_euler = ToEulerAngles(tempQuaternion);

    tempQuaternion.w = link.orientation.w;
    tempQuaternion.x = link.orientation.x;
    tempQuaternion.y = link.orientation.y;
    tempQuaternion.z = link.orientation.z;
    EulerAngles link_euler = ToEulerAngles(tempQuaternion);
    
    link_ori.position.x = link.position.x;
    link_ori.position.y = link.position.y;
    link_ori.position.z = link.position.z;
    link_ori.eular = link_euler;

    tf_ori.position.x = model.position.x - link.position.x;
    tf_ori.position.y = model.position.y - link.position.y;
    tf_ori.position.z = model.position.z - link.position.z;
    tf_ori.eular.roll = model_euler.roll - link_euler.roll;
    tf_ori.eular.pitch = model_euler.pitch - link_euler.pitch;
    tf_ori.eular.yaw = model_euler.yaw - link_euler.yaw;


    // cout << "link_ori:(" << link_ori.position.x << "," << link_ori.position.y << "," << link_ori.position.z << ","
    //     << link_ori.eular.roll << "," << link_ori.eular.pitch << "," << link_ori.eular.yaw << ")" << endl;
    // cout << "model:(" << model.position.x << "," << model.position.y << "," << model.position.z << ","
    //     << model_euler.roll << "," << model_euler.pitch << "," << model_euler.yaw << ")" << endl;
    // cout << "tf_ori:(" << tf_ori.position.x << "," << tf_ori.position.y << "," << tf_ori.position.z << ","
    //     << tf_ori.eular.roll << "," << tf_ori.eular.pitch << "," << tf_ori.eular.yaw << ")" << endl;
}


gazebo_msgs::LinkStates linkStates;
void doGripperLinkPose(const gazebo_msgs::LinkStates::ConstPtr& msg_link_pose)
{
    linkStates = *msg_link_pose;
    // for(int i = 0; i < msg_link_pose->name.size(); i++)
    // {
    //     if(msg_link_pose->name[i] == "robot::link_6")
    //     {
    //         cout << "(" << msg_link_pose->pose[i].position.x << "," << msg_link_pose->pose[i].position.y << ","
    //             << msg_link_pose->pose[i].position.z << "),(" << msg_link_pose->pose[i].orientation.w << ","
    //             << msg_link_pose->pose[i].orientation.x << "," << msg_link_pose->pose[i].orientation.y << ","
    //             << msg_link_pose->pose[i].orientation.z << ")" << endl;
    //     }
    // }
}

gazebo_msgs::ModelStates modelStates;
void doModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg_model_states)
{
    modelStates = *msg_model_states;
}


// 储存一开始的变换关系
void save_tf_ori(string object)
{
    for(int i = 0; i < linkStates.name.size(); i++)
    {
        if(linkStates.name[i] == "robot::link_6")
        {
            for(int q = 0; q < modelStates.name.size(); q++)
            {
                if(modelStates.name[q] == object)
                {
                    // 储存
                    set_tri_ori(modelStates.pose[q], linkStates.pose[i]);
                    break;
                }
            }
            break;
        }
    }
}



std_msgs::String msg_obj_last;
string graspedObj;
void doObj(const std_msgs::String::ConstPtr& msg_obj)
{
    if(msg_obj->data != msg_obj_last.data)
    {
        msg_obj_last.data = msg_obj->data;
        int index = msg_obj->data.find("#");
        if(index != string::npos)
        {
            beGrasped = true;
            graspedObj = msg_obj->data.substr(index + 1);
            cout << "抓取的模型:" << graspedObj << endl;
            // index = graspedObj.find("number");
            // if(index != string::npos)
            // {
            //     graspedObj += "_01";
            // }
            // if(graspedObj == "number_1")
            // {
            //     graspedObj = "number1_01";
            // }
            // else if(graspedObj == "coke")
            // {
            //     graspedObj = "coke_can";
            // }
            // else if(graspedObj == "beer")
            // {
            //     graspedObj = "beer_smaller";
            // }
            save_tf_ori(graspedObj);
        }
        else
        {
            beGrasped = false;
        }
    }
}



// 检查角度
float angleCheck(float angle)
{
    if(angle > -M_PI && angle <= M_PI)
    {
        return angle;
    }
    else if(angle > M_PI)
    {
        while(1)
        {
            angle -= 2 * M_PI;
            if(angle <= M_PI)
            {
                break;
            }
        }
    }
    else
    {
        while (1)
        {
            angle += 2 * M_PI;
            if(angle > -M_PI)
            {
                break;
            }
        }
    }
    return angle;
}


gazebo_msgs::ModelState object_state;
void set_model_state(ros::Publisher set_model_state_pub)
{
    EulerPose link_state_current;
    for(int i = 0; i < linkStates.name.size(); i++)
    {
        if(linkStates.name[i] == "robot::link_6")
        {
            Quaternion tempQuaternion;
            tempQuaternion.w = linkStates.pose[i].orientation.w;
            tempQuaternion.x = linkStates.pose[i].orientation.x;
            tempQuaternion.y = linkStates.pose[i].orientation.y;
            tempQuaternion.z = linkStates.pose[i].orientation.z;
            EulerAngles tempEuler = ToEulerAngles(tempQuaternion);
            link_state_current.position.x = linkStates.pose[i].position.x;
            link_state_current.position.y = linkStates.pose[i].position.y;
            link_state_current.position.z = linkStates.pose[i].position.z;
            link_state_current.eular = tempEuler;
            break;
        }
    }


    EulerPose model_state_current;
    model_state_current.position.x = link_state_current.position.x + tf_ori.position.x;
    model_state_current.position.y = link_state_current.position.y + tf_ori.position.y;
    model_state_current.position.z = link_state_current.position.z + tf_ori.position.z;
    model_state_current.eular.roll = link_state_current.eular.roll + tf_ori.eular.roll;
    model_state_current.eular.pitch = link_state_current.eular.pitch + tf_ori.eular.pitch;
    model_state_current.eular.yaw = link_state_current.eular.yaw + tf_ori.eular.yaw;
    model_state_current.eular.roll = angleCheck(model_state_current.eular.roll);
    model_state_current.eular.pitch = angleCheck(model_state_current.eular.pitch);
    model_state_current.eular.yaw = angleCheck(model_state_current.eular.yaw);

    object_state.model_name = graspedObj;
    object_state.pose.position.x = model_state_current.position.x;
    object_state.pose.position.y = model_state_current.position.y;
    object_state.pose.position.z = model_state_current.position.z;
    Quaternion tempQuaternion = ToQuaternion(model_state_current.eular);
    object_state.pose.orientation.w = tempQuaternion.w;
    object_state.pose.orientation.x = tempQuaternion.x;
    object_state.pose.orientation.y = tempQuaternion.y;
    object_state.pose.orientation.z = tempQuaternion.z;
    set_model_state_pub.publish(object_state);


    // cout << "-----------------------------------------------------" << endl;
    // cout << "link_current: (" << link_state_current.position.x << "," << link_state_current.position.y << ","
    //     << link_state_current.position.z << "," << link_state_current.eular.roll << ","
    //     << link_state_current.eular.pitch << "," << link_state_current.eular.yaw << ")" << endl;

    // cout << "tf_ori: (" << tf_ori.position.x << "," << tf_ori.position.y << ","
    //     << tf_ori.position.z << "," << tf_ori.eular.roll << ","
    //     << tf_ori.eular.pitch << "," << tf_ori.eular.yaw << ")" << endl;
    // cout << "model_state_after_cheack: (" << object_state.pose.position.x << "," << object_state.pose.position.y << ","
    //     << object_state.pose.position.z << "," << model_state_current.eular.roll << ","
    //     << model_state_current.eular.pitch << "," << model_state_current.eular.yaw << ")" << endl;
}


void run_model_set(ros::Publisher set_model_state_pub)
{
    ros::Rate rate(200);
    while(ros::ok())
    {
        if(beGrasped)
        {
            set_model_state(set_model_state_pub);
        }
        rate.sleep();
    }
}




int main(int argc, char **argv)
{   
    // 初始化ROS节点
    ros::init(argc, argv, "grasp_obj");
    ros::NodeHandle nh;
    ros::Subscriber linkStates_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, doGripperLinkPose);
    ros::Subscriber modelStates_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, doModelStates);
    ros::Subscriber grasp_obj_sub = nh.subscribe<std_msgs::String>("/object_grasped", 10, doObj);
    ros::Publisher set_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);


    thread run_model_set_thread = thread(run_model_set, set_model_state_pub);

    
    ros::spin();


    return 0;
}