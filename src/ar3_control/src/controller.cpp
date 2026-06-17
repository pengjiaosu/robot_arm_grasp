
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <std_msgs/String.h>
#include <vector>
#include <stdlib.h>
#include <thread>
#define _USE_MATH_DEFINES 
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <numeric>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/ModelState.h>
#include <ros/package.h>

using namespace std;


// 定义一个储存四元数的结构体
struct Quaternion 
{
    double w, x, y, z;
};
// 定义一个储存欧拉角的结构体
struct EulerAngles 
{
    double roll, pitch, yaw;
};
// 定义一个储存yolo图像识别结果的结构体
struct YoloResult
{
    vector<string> classes; //储存当前帧识别到的类别
    vector<vector<int>> center; //储存对应的中心，结果图像中框的中心坐标，坐标在图像坐标系下，以像素表示
};
// 定义一个储存yolo图像识别信息的结构体
struct YoloInfo
{
    vector<string> classes; //这里储存了所有的类别
};
// 定义一个储存yolo图像信息缓存的结构体
struct YoloCache
{
    int maxCout = 50; //最多计数50次
    vector<int> cout; //对当前识别到的类别进行计数，有助于程序规避偶有一帧识别错误的情况
};

YoloResult yolo_result; //当前yolo识别结果
YoloInfo yolo_info; //yolo信息
YoloCache yolo_cache; //yolo缓存
// 激光雷达测距的距离
float gripper_laser_dis;
// 激光雷达测距的置信度
vector<float> confi;
// 标记当前控制的状态，用于反馈给交互接口
std_msgs::String interaction_back_inform;

//欧拉角转化四元数，网上随便找的一个
Quaternion ToQuaternion(EulerAngles Eular)
{
    double cy = cos(Eular.yaw * 0.5);
    double sy = sin(Eular.yaw * 0.5);
    double cp = cos(Eular.pitch * 0.5);
    double sp = sin(Eular.pitch * 0.5);
    double cr = cos(Eular.roll * 0.5);
    double sr = sin(Eular.roll * 0.5);
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}
//四元数转化为欧拉角，网上随便找的一个
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}


// 这里定义了一些vector量的清空函数，程序运行中特定时刻需要清空变量的内容和存储空间（完全清空）
// 清空yolo_result
void clear_yolo_result()
{
    vector<string>().swap(yolo_result.classes); //使用swap清空
    vector<vector<int>>().swap(yolo_result.center);   
}
// 清空yolo_info
void clear_yolo_info()
{
    vector<string>().swap(yolo_info.classes);
}
// 清空yolo_cache
void clear_yolo_cache()
{
    vector<int>().swap(yolo_cache.cout);
}

// 显示yolo_result中的内容，debug使用
void show_yolo_result()
{
    if(yolo_result.classes.size() + yolo_result.center.size() > 0 && yolo_result.classes.size() == yolo_result.center.size())
    {
        cout << "classes:";
        for(int i = 0; i < yolo_result.classes.size(); i++)
        {
            cout << yolo_result.classes[i] << ",";
        }
        cout << "center:";
        for(int i = 0; i < yolo_result.center.size(); i++)
        {
            cout << "[" << yolo_result.center[i][0] << "," << yolo_result.center[i][1] << "],";
        }
        cout << endl;
    }
}

// 激光雷达测距初始化
void laserInit()
{
    // 激光雷达在扫描范围内共有20线
    // 采用正态分布的置信度，越靠近中间的测距数据具有越高的可信度
    // 正态分布函数中线为0，值为1
    // 20线中，最边缘的线（即0线和19线）在该正态分布函数中对应的值为0.1
    float x;
    // 首先将20线分为左右对称的10线，最边缘线对应的x=0.8561，因此1-10线中，横坐标为i*0.08561
    for(int i = 1; i < 11; i++)
    {
        x = (float)i * 0.08561;
        // 正态分布函数为exp[-pi*x^(2)]，即e的-pi乘x的平方的次方
        confi.push_back(pow(M_E, -M_PI * pow(x, 2)));
    }
    // 另一半置信度完全对称，将confi翻转后，拼接在前面
    vector<float> confi_temp = confi;
    reverse(confi_temp.begin(), confi_temp.end());
    confi.insert(confi.begin(), confi_temp.begin(), confi_temp.end());
    // 这样处理后，虽然置信度呈现了正态分布，但是置信度的和并不为1，因此需要归一化
    float confi_sum = accumulate(confi.begin(), confi.end(), 0.000);
    for(int i = 0; i < confi.size(); i++)
    {
        confi[i] /= confi_sum;
    }
}


// yolo缓存数据的处理，自动计数
void yolo_cache_set()
{
    // 判断yolo_info中是否有数据，规避程序故障的情况，一般来说yolo_info一定有数据
    if(yolo_info.classes.size() > 0)
    {
        // yolo_cache中的cout个数是否和yolo_info中的classes个数一致
        // 一般情况下，只有第一帧yolo_info数据到达的时候会执行，此时yolo_cache是空的，所以这里实际是yolo_cache的初始化
        // yolo_cache.cout的size应该yolo_info.classes的size保持一致
        if(yolo_cache.cout.size() < yolo_info.classes.size())
        {
            // 清空一下
            clear_yolo_cache();
            // 全部置0
            for(int i = 0; i < yolo_info.classes.size(); i++)
            {
                yolo_cache.cout.push_back(0);
            }
        }
        // yolo_cache已经被初始化好了，根据yolo_info的情况对每个类别进行计数
        // 遍历yolo_info.classes
        for(int i = 0; i < yolo_info.classes.size(); i++)
        {
            // 判断当前识别结果yolo_result是否有yolo_info.classes[i]这个类别
            auto result = find(yolo_result.classes.begin(), yolo_result.classes.end(), yolo_info.classes[i]);
            // 如果没有找到，说明当前帧没有识别到这个目标，直接把计数置0
            if(result == yolo_info.classes.end())
            {
                yolo_cache.cout[i] = 0;
            }
            // 其他情况，计数+1，并判断一下是否超过最大值
            else
            {
                yolo_cache.cout[i] ++;
                if(yolo_cache.cout[i] > yolo_cache.maxCout)
                {
                    yolo_cache.cout[i] = yolo_cache.maxCout;
                }
            }
        }
    }
}

// yolo识别结果回调函数
// msg_yolo_result中的格式是: classes:XX,XX,XX,center:[Y,Y],[Y,Y],[Y,Y],
void doYoloResult(const std_msgs::String::ConstPtr& msg_yolo_result)
{
    // ROS_INFO("%s", msg_yolo_result->data.c_str());
    // 使用一个string接收，判断非空进行字符串拆分
    string temp = msg_yolo_result->data;
    if(!temp.empty())
    {
        // 字符串寻找，找到classes和center的索引位置
        int classes_index = temp.find("classes");
        int center_index = temp.find("center");
        // 判断如果找到的话，就开始进行字符串处理，实际上一般情况下一定可以找到classes和center，而且classes_index应该为0
        if(classes_index != string::npos && center_index != string::npos)
        {
            // 新的一帧数据，首先清空之前的识别结果
            clear_yolo_result();
            // 分割字符串，substr函数功能为从一个字符串中取出一部分子字符串，传入两个参数依次为开始的索引和长度
            string classes_str = temp.substr(classes_index, center_index - classes_index);
            // 如果只传入一个参数，则从传入索引开始，一直截取到末尾
            string center_str = temp.substr(center_index);
            // 分割classes，8为"classes:"的长度，这样loc1为第一个类别的开始
            int loc1 = classes_index + 8;
            // 开始循环截取
            while(ros::ok())
            {
                // 寻找loc1之后第一个","的索引
                int loc2 = classes_str.find(",", loc1);
                if(loc2 != string::npos)
                {
                    // 截取获得第一个类别的名称
                    yolo_result.classes.push_back(classes_str.substr(loc1, loc2 - loc1));   
                    // 指向下一个类别的开始
                    loc1 = loc2 + 1;
                }
                // 如果找不到了，说明已经到达字符串结尾，已经寻找完毕，退出循环
                else
                {
                    break;
                }
            }
            // 分割center
            // 寻找"["的位置，+1指向第一个坐标的x的第一个数字
            loc1 = center_str.find("[") + 1;
            if(loc1 != string::npos)
            {
                // 循环截取
                while(ros::ok())
                {
                    // 寻找loc1之后第一个","的索引
                    int loc2 = center_str.find(",", loc1);
                    if(loc2 != string::npos)
                    {
                        // atoi将字符串转换为int，需要传入c风格字符串，center_str.substr(loc1, loc2 - loc1)截取获得了字符串格式的坐标的x的值，
                        // .c_str()将string转换为c风格字符串char类型
                        int x = atoi(center_str.substr(loc1, loc2 - loc1).c_str());
                        // 指向y的第一个数字
                        loc1 = loc2 + 1;
                        // 寻找loc1之后第一个"]"的索引
                        loc2 = center_str.find("]", loc1);
                        if(loc2 != string::npos)
                        {
                            // 同理获取y的int
                            int y = atoi(center_str.substr(loc1, loc2 - loc1).c_str());
                            // 储存在yolo_result中
                            yolo_result.center.push_back({x,y});
                            // 3是"],["的长度，这样直接指向了下一个坐标的x的第一个数字
                            loc1 = loc2 + 3;
                        }
                        // 一般情况下不会从这里退出循环
                        else
                        {
                            break;
                        }
                    }
                    // 找不到","，说明已经到达字符串结尾，已经寻找完毕，退出循环
                    else
                    {
                        break;
                    }
                }
            }            
        }
        // 根据当前识别的结果更新一次yolo_cache
        yolo_cache_set();
    }
    // show_yolo_result();
}


// yolo info的回调函数
// msg_yolo_info中的格式为：all classes name:XX,XX,XX
void doYoloInfo(const std_msgs::String::ConstPtr& msg_yolo_info)
{
    
    string temp = msg_yolo_info->data;
    // 寻找all classes name
    int all_classes_name_index = temp.find("all classes name");
    if(all_classes_name_index != string::npos)
    {
        // 清空之前的yolo_info
        clear_yolo_info();
        // 17为"all classes name:"的长度，这样loc1为第一个类别开始的索引
        int loc1 = all_classes_name_index + 17;
        // 循环处理
        while(ros::ok())
        {
            // 寻找","
            int loc2 = temp.find(",", loc1);
            if(loc2 != string::npos)
            {
                // 设置yolo_info
                yolo_info.classes.push_back(temp.substr(loc1, loc2 - loc1));
                // 指向下一个类别的开始
                loc1 = loc2 + 1;
            }
            // 处理完成，结束循环
            else
            {
                break;
            }
        }
    }
}

// 激光雷达数据回调函数
void doLaser(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    sensor_msgs::LaserScan laserData = *laser;
    gripper_laser_dis = 0;
    // 将每一线的测得距离乘上对应的置信度，再累加，即得到一个算术平均值，作为可信的距离
    for(int i = 0; i < laserData.ranges.size(); i++)
    {
        gripper_laser_dis += laserData.ranges[i] * confi[i];
    }
}


string interaction_msg; //交互窗口传回的信息，格式示例为：#0$2，表示抓取number0，放置到2号位置
string numberBeGrasper_str; // 需要抓取的数字box的模型对应的标号
int numberRelesedLocalMark; // 需要放置的位置
// 获取需要抓取的数字回调函数
void doInteractionInput(const std_msgs::String::ConstPtr& msg_interaction)
{
    // 储存消息到interaction_msg
    interaction_msg = msg_interaction->data;
    // 消息不是"None"并且非空
    if(interaction_msg != "None" && interaction_msg.size() > 0)
    {
        // 先找到#和$
        int index1 = msg_interaction->data.find("#");
        int index2 = msg_interaction->data.find("$");
        if(index1 != string::npos && index2 != string::npos)
        {
            // 分割储存
            numberBeGrasper_str = msg_interaction->data.substr(index1 + 1, 1);
            numberRelesedLocalMark = atoi(msg_interaction->data.substr(index2 + 1, 1).c_str());
        }
    }
}


// 运行ros的回调函数，单独线程启动
void runRosCallBack()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}


// 机械臂控制的class
class ArmController
{
public:
    // 初始化
    ArmController(ros::Publisher grasp_obj_pub_, ros::Publisher set_model_state_pub_, ros::NodeHandle nh_)
    {
        
        ros::AsyncSpinner spinner(1);
        spinner.start();

        // 接收部分参数
        set_model_state_pub = set_model_state_pub_; //设置Gazebo中模型状态的话题发布
        grasp_obj_pub = grasp_obj_pub_; //需要抓取的模型名称的话题发布
        nh = nh_; //ros NodeHandle

        // 进行一些初始化操作
        // 初始化number_init_loc，number_init_loc中储存了number0-number9共10个模型的初始位姿，每个位姿包括6个变量，分别为x,y,z,roll,pitch,yaw
        for(int i = 0;; i++)
        {
            number_init_loc.push_back({0, 0, 0, 0, 0, 0});
            // 从src/ar3_control/config/numberi_init.yaml文件中加载坐标
            if(nh.getParam("number" + to_string(i), number_init_loc[i]))
            {
                numberBeMovedTimes.push_back(0);
            }
            // 从number10开始，找不到这个变量了，退出循环
            else
            {
                break;
            }
        }
        // 初始化numberReleaseLoc，numberReleaseLoc中储存1-6号放置位置的x,y轴坐标，1-6号位置对应release_loc0-release_loc5
        for(int i = 0;; i++)
        {
            numberReleaseLoc.push_back({0, 0});
            if(nh.getParam("release_loc" + to_string(i), numberReleaseLoc[i]))
            {
                numberReleaseOcc.push_back(false);
            }
            else
            {
                break;
            }
        }
        // 模型位置初始化
        modelAllInit();
        // 控制机械臂回到"home"位姿
        pose_home();
    }

    // 控制机械臂回到初始化位置
    void pose_home()
    {
        // "home"位姿是在moveit中预设的一个位姿
        arm.setNamedTarget("home");
        arm.move(); //规划+运动
        ros::Duration(1).sleep();
    }

    // 搜索并抓取目标
    bool searchAndGrabTarget(string name)
    {
        // 将返回给交互窗口的数据设置为"Search"，标志进入搜索阶段
        interaction_back_inform.data = "Search";
        // 输出搜索目标
        ROS_INFO("搜索目标: %s", name.c_str());
        // 声明两个变量，和机械臂搜索有关
        int step = 1;
        int num;
        // 搜索开始后，机械臂会在圆弧的一定区间内往复运动一次，如果搜索到目标则停止搜索动作，进入抓取状态
        // 如果往复运动一次后没有进入抓取状态，则说明没有找到目标

        // 声明一个机械臂末端执行器在世界坐标系下的位姿变量
        geometry_msgs::Pose target_pose;
        // 声明一个bool量，标志是否完成目标抓取
        bool done_grip_target = false;
        // 生命一个bool量，标记是否需要开始检测目标
        bool start_check_target = false;
        // 左右摆动一次搜索目标
        for(num = 8; num >= 8; num += step)
        {
            // 设置末端执行器位姿为搜索路径的起点
            target_pose = circle_pose(num, 15, 0.4, 0.4);
            arm.setPoseTarget(target_pose);
            // 调用逆运动学求解器计算关节角度
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
            {
                // ROS_INFO("Planning Successful!");
                // 执行规划的轨迹
                arm.execute(my_plan);
                // 从home进入搜索路径时，先暂停1s，再允许目标检测
                // 这是因为在机械臂从home到路径第一个点的过程中，可能出现目标识别误检测的情况，因此暂停1s规避
                if(!start_check_target)
                {
                    ros::Duration(1).sleep();
                    start_check_target = true;
                }
            }
            else
            {
                // 如果当前路径点的规划失败，则直接规划下一个点，一般情况下不会这样
                num += step;
            }
            // 完成第一次规划后才开始检查目标，也可以说是进入路径后才开始目标检测
            if(start_check_target)
            {
                // 判断是否在当前摄像头图像中找到了需要寻找的目标
                bool find_target = check_target(name);
                if(find_target)
                {
                    ROS_INFO("找到目标: %s", name.c_str());
                    // 如果找到了目标，则开始对正，因为目标出现在摄像头图像中的时候，一般是在图像的边角位置，对正是指将机械臂末端对准目标，即将目标在摄像头图像的中心
                    // 进入对正和抓取阶段
                    bool aim_target_success = aim_target(name, target_pose);
                    // 返回true值，说明完成抓取
                    if(aim_target_success)
                    {
                        return true;
                    }
                    // 否则搜索失败，返回false
                    else
                    {
                        return false;
                    }
                }
            }
            // num可以在10-19内变化，如果num增加到了19，则将step修改为-1，num开始减少
            if(num == 40)
            {
                step = -1;
            }
            // ros::Duration(2).sleep();
        }
        // num从10增加到19又减少到9，说明完成一次往复搜索，退出循环，没有搜素到目标
        ROS_INFO("没有搜索到目标");
        // 标记任务失败
        interaction_back_inform.data = "Failed";
        // 返回false
        return false;
    }

    // 抓取并移动
    void numberMove()
    {
        // 一般来说numberBeGrasper_str的长度只能是1，判断规避偶然的错误情况
        if(numberBeGrasper_str.size() == 1)
        {
            // 拼接一下字符串，还原具体的模型名称
            string number_name = "number" + numberBeGrasper_str;
            // 搜索并抓取目标
            searchAndGrabTarget(number_name);
        }
    }

    // 模型全部初始化
    void modelAllInit()
    {
        // 将标记模型被移动变量的vector中全部置0，认为模型没有被抓取过
        for(int i = 0; i < numberBeMovedTimes.size(); i++)
        {
            numberBeMovedTimes[i] = 0;
        }
        // 将放置位置的被占据状态也设置为0，认为放置位置没有被占用过
        for(int i = 0; i< numberReleaseOcc.size(); i++)
        {
            numberReleaseOcc[i] = false;
        }

        // 一次设置number0-number9的位置
        // 为了保证在多次连续抓取中能一直连续抓取同一个数字，比如最多可以抓取6次number0
        // 仿真场景中实际上已经添加了6套number0-number9的模型，只不过一开始只有一套模型在机械臂可以抓取到的范围里面
        // 其余模型都在场景中其他地方，当一个模型被抓取挪走后，直接从场景中将这个模型的另外一套对应的模型挪过来
        // 模型的命名规则可以在gazebo中看到，
        // 例如number0，一共有6个模型，Gazebo中的名称分别是number0_01, number0_01_clone, number0_01_clone_0, number0_01_clone_1, number0_01_clone_2, number0_01_clone_3
        for(int i = 0; i < 10; i++)
        {   
            // xy标记了放置的起点为(8,8)位置
            float x = 8;
            float y = 8;
            // extern_str中储存了number0字符以外的额外的字符，拼接后为完整的模型名称
            for(int q = 0; q < extern_str.size(); q++)
            {
                // 声明一个gazebo模型状态消息
                gazebo_msgs::ModelState object_state;
                // 拼接出完整的名称
                object_state.model_name = "number" + to_string(i) + extern_str[q];
                // 如果是number0_01，说明是应该出现在机械臂可以抓取范围内的第一套模型，则放置在模型的初始位置
                if(extern_str[q] == "_01")
                {
                    object_state.pose.position.x = number_init_loc[i][0];
                    object_state.pose.position.y = number_init_loc[i][1];
                }
                else
                {
                    // 其他套模型挪到场景的其他地方
                    object_state.pose.position.x = x + 0.2 * i;
                    object_state.pose.position.y = y + 0.2 * q;
                }
                object_state.pose.position.z = number_init_loc[i][2];
                // 获取欧拉角
                EulerAngles eular;
                eular.roll = number_init_loc[i][3];
                eular.pitch = number_init_loc[i][4];
                eular.yaw = number_init_loc[i][5];
                // 将欧拉角转换为四元数，并设置
                Quaternion quaternion = ToQuaternion(eular);
                object_state.pose.orientation.w = quaternion.w;
                object_state.pose.orientation.x = quaternion.x;
                object_state.pose.orientation.y = quaternion.y;
                object_state.pose.orientation.z = quaternion.z;
                // 发送5次，确保Gazebo接收到信息并完成模型设置
                for(int i = 0; i < 5; i++)
                {
                    set_model_state_pub.publish(object_state);
                    ros::Duration(0.01).sleep();
                }
            }
        }
    }
private:
    // 设置模型位置的发布话题
    ros::Publisher set_model_state_pub;
    // ros NodeHandle
    ros::NodeHandle nh;
    // 机械臂附近的模型的初始位置
    vector<vector<float>> number_init_loc;
    // 数字模型被移动的次数记录，涉及模型的具体名称判断
    vector<int> numberBeMovedTimes;
    // 模型被释放的位置，答案区域最多可以防止3个模型，储存了这个3个位置的坐标信息
    vector<vector<float>> numberReleaseLoc;
    // 模型释放位置的占用情况，记录了答案区域3个位置是否被占据，用于判断模型应该防止在哪个位置
    vector<bool> numberReleaseOcc;
    // 发布需要抓取的模型名字，发布给抓取功能包，以实现抓取效果仿真
    ros::Publisher grasp_obj_pub;
    // 创建MoveGroupInterface对象
    moveit::planning_interface::MoveGroupInterface arm = moveit_arm_init(); 
    // 储存还原模型完整名称需要额外拼接的字符串
    vector<string> extern_str = {"_01_clone_3", "_01_clone_2", "_01_clone_1", "_01_clone_0", "_01_clone", "_01"};
    // 机械臂初始化
    moveit::planning_interface::MoveGroupInterface moveit_arm_init()
    {
        // 初始化规划类
        moveit::planning_interface::MoveGroupInterface arm("arm");
        arm.setGoalPositionTolerance(0.01); // 容许位置误差
        arm.setGoalOrientationTolerance(0.01); // 容许旋转角误差
        arm.setMaxAccelerationScalingFactor(0.5); // 最大加速度
        arm.setMaxVelocityScalingFactor(0.5); // 最大速度
        return arm;
    }
    // 对正目标的子函数
    bool aim_target(string name, geometry_msgs::Pose target_pose)
    {
        // 将反馈给交互窗口的信息设置为"Aim"，标志进入对正阶段
        interaction_back_inform.data = "Aim";
        // 设置对正允许的像素误差，这里是30个像素
        int diff_Tolerance = 30;
        // 进入循环
        while(ros::ok())
        {
            // 声明一个目标索引，对应目标在yolo_result.classes中的位置，初始化为-1，如果没有找到目标，则会保持-1的值
            int target_index = -1;
            // 遍历yolo_result.classes寻找name
            for(int i = 0; i < yolo_result.classes.size(); i++)
            {
                // 如果找到，则target_index为对应的索引，并结束for循环
                if(yolo_result.classes[i] == name)
                {
                    target_index = i;
                    break;
                }
            }
            // 如果target_index的值不为-1，说明在当前yolo识别结果中找到了需要抓取的目标，则尝试对正
            if(target_index != -1)
            {
                // 项目中相机的分辨率为1280*720，因此图像中心的像素坐标为(640,360)
                // 注意为图像坐标系，和一般的直角坐标系不同
                // 计算x和y轴的当前像素差距
                int diff_x = yolo_result.center[target_index][0] - 640;
                int diff_y = yolo_result.center[target_index][1] - 360;
                // 如果两个轴的像素差距都小于允许的误差，则认为已经对正目标
                if(abs(diff_x) < diff_Tolerance && abs(diff_y) < diff_Tolerance)
                {
                    // 进入抓取阶段
                    bool grabSuccess = grabTarget(name, target_pose);
                    return true;
                }
                // 如果没有对正，则继续挪动机械臂末端，直到对正
                else
                {
                    // 根据像素差距设置机械臂末端的位姿，注意图像坐标系和Gazebo世界坐标系的方向转换
                    target_pose.position.x += (-1) * diff_x * 0.0002;
                    target_pose.position.y += diff_y * 0.0002;
                    // 为了避免特殊情况（虽然一般不会出现），避免目标点设置到了机械臂不可到达的位置
                    // 计算当前目标点距离原点的平面距离，因为机械臂放置在坐标原点，所以这个值也是机械臂需要伸长的距离
                    float R = pow(pow(target_pose.position.x, 2) + pow(target_pose.position.y, 2), 0.5);
                    // 最远允许伸长0.5m，如果超过，则只伸长0.5m
                    if(R > 0.5)
                    {
                        target_pose.position.x *= (0.5 / R);
                        target_pose.position.y *= (0.5 / R);
                    }
                    // 逆运动学阶段并驱动
                    arm.setPoseTarget(target_pose);
                    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                    bool success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    if (success)
                    {
                        // 执行规划的轨迹
                        arm.execute(my_plan);
                    }
                }
            }
            // 如果target_index的值为-1，说明当前没有找到目标，但是能够进入对正阶段，说明在当前搜索周期内肯定是看到过目标的，只是现在丢失了
            // 因此做丢失处理
            else
            {
                bool find_target_again = lost_target(name);
                // 如果丢失处理后依然找不到目标，则只能宣告搜索失败
                if(!find_target_again)
                {
                    return false;
                }
            }
        }
        return false;
    }
    // 目标丢失后的处理
    bool lost_target(string name)
    {
        ROS_INFO("目标丢失，等待5s");
        // 等待5s的时间，等待yolo识别图像，如果依然找不到目标，则目标丢失
        float lost_target_time = ros::Time::now().toSec();
        ros::Rate rate(30);
        // 循环等待
        while (ros::ok())
        {
            // 检查yolo识别结果中是否有需要抓取的模型
            bool find_target = check_target(name);
            // 如果找到目标，则继续对正
            if(find_target)
            {
                ROS_INFO("重新找到目标: %s", name.c_str());
                return true;
            }
            // 超过5s，退出
            if(ros::Time::now().toSec() - lost_target_time > 5)
            {
                return false;
            }
            rate.sleep();
        }
        return false;
    }
    // 抓取目标
    bool grabTarget(string name, geometry_msgs::Pose target_pose)
    {
        // 将反馈给交互窗口的信息设置为"Grasp"，标志进入抓取阶段
        interaction_back_inform.data = "Grasp";

        // 声明机械臂末端到模型的距离
        float expect_z = 0.03;
        // 声明允许的误差
        float diff_Tolerance = expect_z / 10;
        // 进入循环
        while(ros::ok())
        {
            // 根据激光雷达反馈的距离，修正机械臂末端位姿
            target_pose.position.z += (expect_z - gripper_laser_dis) * 0.5;
            arm.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
            {
                // 执行规划的轨迹
                arm.execute(my_plan);
            }
            // cout << "距离:" << gripper_laser_dis << endl;
            // 如果距离差小于容许误差
            if(abs(gripper_laser_dis - expect_z) < diff_Tolerance)
            {
                ROS_INFO("抓取");
                // 声明一个std_msgs::String，准备发送给抓取目标的程序
                std_msgs::String graspModel;
                string model_extern_str = "";
                int index = name.find("number");
                int num;
                if(index != string::npos)
                {
                    // 获取需要抓取模型对应的数字，以及还原模型名称的字符串
                    num = atoi(name.substr(index + 6).c_str());
                    model_extern_str = extern_str[extern_str.size() - 1 - numberBeMovedTimes[num]];
                }
                // 组织消息，消息格式是#+模型名称
                graspModel.data = "#" + name + model_extern_str;
                // 发送10次，确保接收到
                for(int times = 0; times < 10; times++)
                {
                    grasp_obj_pub.publish(graspModel);
                    // 延时0.1s
                    ros::Duration(0.1).sleep();
                }
                // 已经设置了抓取，模型会跟随机械臂末端运动，现在将机械臂抬起
                target_pose.position.z = 0.4;
                arm.setPoseTarget(target_pose);
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                bool success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (success)
                {
                    arm.execute(my_plan);
                }
                // 确保抬起完成
                ros::Duration(1).sleep();
                // 移动模型到设置的位置并释放
                MoveAndReleaseNumber(target_pose);
                // 完成移动，标记该模型被移动次数+1
                numberBeMovedTimes[num] ++;
                // 重新移动一个新的模型过来
                if(numberBeMovedTimes[num] < extern_str.size())
                {
                    // 现在name为当前数字模型的下一套模型名称
                    name += extern_str[extern_str.size() - 1 - numberBeMovedTimes[num]];
                }
                // 移动模型到机械臂可以抓取的初始点
                moveModel2Init(name);
                // 执行成功，返回true
                return true;
            }
        }
        return false;
    }
    // 移动模型到设定的位置并释放
    bool MoveAndReleaseNumber(geometry_msgs::Pose target_pose)
    {
        // 将反馈给交互窗口的信息设置为"Move"，标志进入移动阶段
        interaction_back_inform.data = "Move";
        // 移动，根据需要放置的位置获取对应坐标，并设置机械臂的末端位置
        target_pose.position.x = numberReleaseLoc[numberRelesedLocalMark - 1][0];
        target_pose.position.y = numberReleaseLoc[numberRelesedLocalMark - 1][1];
        target_pose.position.z = 0.4;
        arm.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            arm.execute(my_plan);
        }
        // 延时1s，确保移动完成
        ros::Duration(1).sleep();
        // 降低高度
        // 释放模型的高度为0.25m
        target_pose.position.z = 0.18;
        arm.setPoseTarget(target_pose);
        success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            arm.execute(my_plan);
        }
        ros::Duration(1).sleep();

        // 释放
        // 标志进入释放阶段
        interaction_back_inform.data = "Release";
        ROS_INFO("放下");
        // 发布"None"给抓取模型的程序，标志停止抓取
        std_msgs::String graspModel;
        graspModel.data = "None";
        // 发送10次，确保接收到信息
        for(int times = 0; times < 10; times++)
        {
            grasp_obj_pub.publish(graspModel);
            ros::Duration(0.1).sleep();
        }
        // 标记该位置为占据
        numberReleaseOcc[numberRelesedLocalMark] = true;
        return true;
    }

    // 检查yolo识别结果中是否包括需要搜索的目标
    bool check_target(string name)
    {
        // 遍历当前的yolo识别结果
        for(int i = 0; i < yolo_result.classes.size(); i++)
        {
            // 如果在yolo识别结果中找到目标
            if(yolo_result.classes[i] == name)
            {
                // 在yolo_info.classes中找到目标，确认是否在之前的图像中也识别到了该目标，排除偶然的识别错误
                auto result = find(yolo_info.classes.begin(), yolo_info.classes.end(), name);
                if(result != yolo_info.classes.end())
                {
                    // 获取索引
                    int index = distance(yolo_info.classes.begin(), result);
                    // 如果计数大于10，说明在之前连续10帧图像中都找了该目标，排除偶然识别错误的情况，认为确实找到了目标
                    if(yolo_cache.cout[index] >= 10)
                    {
                        // 返回true
                        return true;
                    }
                }
            }
        }
        // 否则返回false
        return false;
    }
    //实例，画半圆,获取末端的位姿，num为起始位置，precision为精度，height为移动高度，r为半径
    geometry_msgs::Pose circle_pose(int num, double precision, double height, double r)
    {
        num = (double)num;
        // 设置欧拉角，在移动过程中，机械臂末端位姿保持不变
        EulerAngles targetEular;
        targetEular.roll = 0;
        targetEular.pitch = M_PI / 2;
        targetEular.yaw = -M_PI / 2;

        // 计算角度
        double angle = (((180 / precision) * num) / 180) * M_PI;
    
        // 根据角度和半径计算位置，添加-是为了与Gazebo中的坐标系对应
        geometry_msgs::Pose target_pose;
        target_pose.position.x = r * cos(angle);  // X位置
        target_pose.position.y = -r * sin(angle);  // Y位置
        target_pose.position.z = height;  // Z位置
        // 将欧拉角转换为四元数
        Quaternion targetQuaternion = ToQuaternion(targetEular);
        target_pose.orientation.x = targetQuaternion.x;
        target_pose.orientation.y = targetQuaternion.y;
        target_pose.orientation.z = targetQuaternion.z;
        target_pose.orientation.w = targetQuaternion.w;
        return target_pose;
    }
    // 移动一个模型到初始位置
    void moveModel2Init(string model)
    {
        int index = model.find("number");
        if(index != string::npos)
        {
            // 获取对应的数字
            int num = atoi(model.substr(index + 6, 1).c_str());
            gazebo_msgs::ModelState object_state;
            object_state.model_name = model;
            // 设置初始位姿
            object_state.pose.position.x = number_init_loc[num][0];
            object_state.pose.position.y = number_init_loc[num][1];
            object_state.pose.position.z = number_init_loc[num][2];
            EulerAngles eular;
            eular.roll = number_init_loc[num][3];
            eular.pitch = number_init_loc[num][4];
            eular.yaw = number_init_loc[num][5];
            Quaternion quaternion = ToQuaternion(eular);
            object_state.pose.orientation.w = quaternion.w;
            object_state.pose.orientation.x = quaternion.x;
            object_state.pose.orientation.y = quaternion.y;
            object_state.pose.orientation.z = quaternion.z;
            // 发送10次，确保接收
            ros::Rate rate(10);
            for(int i = 0; i < 10; i++)
            {
                set_model_state_pub.publish(object_state);
                rate.sleep();
            }
        }
    }
};


// yaml加载，在ros节点生成之前操作
void yaml_init()
{
    // 获取ar3_control功能包的路径
    string package_path = ros::package::getPath("ar3_control");
    // 拼接字符串，指向config目录下的yaml文件
    string yaml_path = package_path + "/config/numberi_init.yaml";
    // 加载yaml文件
    system(("rosparam load " + yaml_path).c_str());
}

// 发送返回消息给交互接口，单独线程启动
void backInform2Interaction(ros::Publisher inform_back_interaction_pub)
{
    // 低速循环发送
    ros::Rate rate(10);
    while(ros::ok())
    {
        inform_back_interaction_pub.publish(interaction_back_inform);
        rate.sleep();
    }
}


// 主函数
int main(int argc, char** argv)
{
    // 正确显示中文
    setlocale(LC_ALL, "");
    // 加载yaml文件
    yaml_init();
    // 初始化ROS节点
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    laserInit(); //激光雷达初始化，计算置信度
    // 初始化订阅话题
    ros::Subscriber yolo_resuly_sub = nh.subscribe<std_msgs::String>("/yolo_result", 10, doYoloResult);
    ros::Subscriber yolo_info_sub = nh.subscribe<std_msgs::String>("/yolo_info", 10, doYoloInfo);
    ros::Subscriber gripper_laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/gripper/laser/scan", 10, doLaser);
    ros::Subscriber number_grasped_sub = nh.subscribe<std_msgs::String>("/interaction_input", 10, doInteractionInput);
    // 初始化发布话题
    ros::Publisher grasp_obj_pub = nh.advertise<std_msgs::String>("/object_grasped", 10);
    ros::Publisher set_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    ros::Publisher inform_back_interaction_pub = nh.advertise<std_msgs::String>("/interaction_back", 10);
    // 单独线程处理ros话题回调函数，避免主线程干扰
    thread ros_callback_thread = thread(runRosCallBack);
    // 单独线程启动交互接口消息回传
    interaction_back_inform.data = "Init"; //标记初始化完成
    thread backInform2Interaction_thread = thread(backInform2Interaction, inform_back_interaction_pub);
    // 创建机械臂控制类
    ArmController controller(grasp_obj_pub, set_model_state_pub, nh);
    interaction_back_inform.data = "Ready"; //标记机械臂准备完成
    cout << "初始化完成" << endl;
    // 主循环开始
    while(ros::ok())
    {
        // 如果交互窗口的消息非空且不为"None"，说明需要进行操作
        if(interaction_msg.size() > 0 && interaction_msg != "None")
        {
            // 如果是"Init"，说明需要初始化，重置模型
            if(interaction_msg == "Init")
            {
                interaction_back_inform.data = "Init";
                controller.modelAllInit();
                interaction_back_inform.data = "Ready";
                cout << "初始化完成" << endl;
            }
            // 其他情况，说明用户输入了需要抓取的模型和放置的位置，开始搜索模型
            else
            {
                controller.numberMove();
                // 执行完成后，机械臂回到"home"位置
                controller.pose_home();
                // 暂停10s
                ROS_INFO("本次任务完成，暂停10s后重置");
                int pause_time = 5;
                for(int time = 0; time < pause_time; time++)
                {
                    cout << to_string(pause_time - time) << "..." << endl;
                    ros::Duration(1).sleep();
                }
                ROS_INFO("等待下一次任务");
                interaction_back_inform.data = "Ready";
            }

        }
        // 循环等到0.1s，大致10Hz
        ros::Duration(0.1).sleep();
    }
    // 程序退出
    ros::shutdown();
    return 0;
}
