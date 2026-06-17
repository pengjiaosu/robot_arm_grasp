
import rospy
import os
import cv2
import time
import argparse
import sys
import rospkg

# 获取功能包的路径
currentRospackPath = rospkg.RosPack().get_path('yolo')
# 指向yolo的源码文件，确保之后能够正确import
yolo_path = currentRospackPath + "/scripts/Yolo-FastestV2"
sys.path.append(yolo_path)


import torch
import model.detector
import utils.utils
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from threading import Thread



# 订阅图像话题的回调函数，也是推理函数
def img_forward_process(ros_img):
    # 全局变量使用声明
    global yolo_model, device, cfg, yolo_classes, yolo_center, yolo_result_img  
    # ros格式的图像转换为opencv的图像，送入推理
    ori_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
    #数据预处理
    res_img = cv2.resize(ori_img, (cfg["width"], cfg["height"]), interpolation = cv2.INTER_LINEAR) 
    img = res_img.reshape(1, cfg["height"], cfg["width"], 3)
    img = torch.from_numpy(img.transpose(0,3, 1, 2))
    img = img.to(device).float() / 255.0

    #模型推理
    start = time.time()
    preds = yolo_model(img)
    end = time.time()
    spend_time = (end - start) * 1000.
    # print("forward time:%fms"%spend_time, end='---')

    #特征图后处理
    output = utils.utils.handel_preds(preds, cfg, device)
    output_boxes = utils.utils.non_max_suppression(output, conf_thres = 0.3, iou_thres = 0.4)

    #加载label names
    LABEL_NAMES = []
    with open(cfg["names"], 'r') as f:
        for line in f.readlines():
            LABEL_NAMES.append(line.strip())

    h, w, _ = ori_img.shape
    scale_h, scale_w = h / cfg["height"], w / cfg["width"]

    yolo_classes = []
    yolo_center = []

    #绘制预测框
    for box in output_boxes[0]:
        box = box.tolist()
        obj_score = box[4]
        if float(obj_score) > 0.95:
            category = LABEL_NAMES[int(box[5])]
            
            x1, y1 = int(box[0] * scale_w), int(box[1] * scale_h)
            x2, y2 = int(box[2] * scale_w), int(box[3] * scale_h)
            center = [(x1 + x2) / 2, (y1 + y2) / 2]
            # print("class: %s, center: (%d, %d)" %(category, center[0], center[1]), end='')
            
            # 将识别到的类别和对应的中心置入列表
            yolo_classes.append(category)
            yolo_center.append([center[0], center[1]])

            cv2.rectangle(ori_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(ori_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)	
            cv2.putText(ori_img, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
    # 将推理结果转换回ros图像
    yolo_result_img = bridge.cv2_to_imgmsg(ori_img, encoding="bgr8")


# 发布识别结果
def publishYoloResult():
    # 全局变量声明
    global yolo_result_pub, yolo_classes, yolo_center, yolo_result_img_pub, yolo_result_img
    # yolo_info可以低速发布，减少对带宽占用和系统负担，这里设置1Hz
    frequence = 50 #图像发布循环频率50Hz
    # 计数
    cout = 0
    rate = rospy.Rate(frequence)
    # 循环发布图像
    while not rospy.is_shutdown():
        # yolo_result组织，按照格式拼接字符串
        yolo_result = String()
        yolo_result.data = "classes:"
        for name in yolo_classes:
            yolo_result.data += "%s," %(name)
        yolo_result.data += "center:"
        for center in yolo_center:
            yolo_result.data += "[%s,%s]," %(str(int(center[0])), str(int(center[1])))
        # 偶有出现一些错误的空格、换行符等，去除
        yolo_result.data.replace("\n", "")
        yolo_result.data.replace("\r", "")
        yolo_result.data.replace(" ", "")
        # 发布识别结果
        yolo_result_pub.publish(yolo_result)
        # 1Hz发布yolo_info
        if cout % frequence == 0:
            yolo_info_pub.publish(all_classes_name)
        # 发布识别结果图像
        yolo_result_img_pub.publish(yolo_result_img)
        # 计数+1
        cout += 1
        rate.sleep()



if __name__ == '__main__':

    rospy.init_node("yolo_detect")
    opt = argparse.Namespace()
    # 指定data和weights文件的路径
    opt.data = currentRospackPath + '/scripts/Yolo-FastestV2/data/number.data'
    opt.weights = currentRospackPath + '/scripts/Yolo-FastestV2/weights/number.pth'

    cfg = utils.utils.load_datafile(opt.data)
    assert os.path.exists(opt.weights), "请指定正确的模型路径"
    # 指向.names文件，里面储存了所有可以识别的类别
    cfg["names"] = currentRospackPath + "/scripts/Yolo-FastestV2/" + cfg["names"]
    #加载获取所有classes的name，储存在列表classes_name中
    classes_name = []
    with open(cfg["names"], 'r') as f:
        for line in f.readlines():
            classes_name.append(line.strip())
    # 组织一个消息，作为yolo_info，发送给controller，告诉其所有的类别
    all_classes_name = String()
    all_classes_name.data = "all classes name:"
    for i in range(0, len(classes_name)):
        all_classes_name.data += classes_name[i] + ","
    # 模型加载
    # 检查cuda，有gpu则使用gpu，没有就使用cpu
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # 传入参数和模型
    yolo_model = model.detector.Detector(cfg["classes"], cfg["anchor_num"], True).to(device)
    yolo_model.load_state_dict(torch.load(opt.weights, map_location=device))

    # sets the module in eval node
    yolo_model.eval()
    # 加载ros cvbridge
    bridge = CvBridge()
    # 订阅图像话题
    sub = rospy.Subscriber("/gripper_camera/image_raw", Image, img_forward_process, queue_size=10)
    # 创建话题发布
    yolo_info_pub = rospy.Publisher("/yolo_info", String, queue_size=10)
    yolo_result_pub = rospy.Publisher("/yolo_result", String, queue_size=10)
    yolo_result_img_pub = rospy.Publisher("/yolo_result_img", Image, queue_size=10)
    # 声明一个ros图像用于发送
    yolo_result_img = Image()
    # 声明列表，分别用于储存类别和其在图像中对应的中心点
    yolo_classes = []
    yolo_center = []
    # 独立线程发送结果
    pub_thread = Thread(target=publishYoloResult)
    pub_thread.daemon = True #保护线程
    pub_thread.start() #线程启动

    # 启动回调
    rospy.spin()


    

        

