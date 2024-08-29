import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import lcm
import toml
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
import threading
import time

class MultiTopicNode(Node):
    def __init__(self):
        super().__init__('multi_topic_node')
        self.lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.msg = robot_control_cmd_lcmt()
        self.steps = toml.load("cyberdog.toml")
        self.num = 0
        self.lastorder = 0
        self.cc = 0
        self.dd = -1
        self.running = True
        self.logger = self.get_logger()
        
    def run(self):
        while self.running:
            self.execute_sequence()  # 添加新的方法来执行一系列操作
            # self.num = 1
            # self.update_and_publish()
            # time.sleep(0.2)

    def execute_sequence(self):

      #执行站起来的动作
        if self.cc == 1:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if self.cc <= 8:
            self.num = 1
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.3)  # 确保有足够的时间发送命令

        # 前进
        if self.cc == 9:
            self.msg.life_count += 1
            print(self.msg.life_count)
        if 8 < self.cc <= 42:
            self.num = 5
            self.cc +=1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4)  # 确保有足够的时间发送命令           

        # 右跳
        if self.cc == 43:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        # # 执行向前right的动作
        if 42 < self.cc <= 67:
            self.right_jump()
        

        # 向后走
        if self.cc == 68:
            self.msg.life_count += 1
            # print(self.msg.life_count)
            
        if 67 < self.cc <= 168:
            self.num = 6
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.45)  # 确保有足够的时间发送命令             

        # 旋转
        if self.cc == 169:
            self.msg.life_count += 1
            print(self.msg.life_count)
        if 168 <self.cc <=208:
            self.circle_stop_180()

        # 下斜坡
        if self.cc == 209:
            self.msg.life_count += 1
            print(self.msg.life_count)
        if 208 < self.cc <= 309:
            self.num = 22
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4) 


        # 左旋转
        if self.cc == 310:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 309 < self.cc <= 349:
            self.circle_stop_90()

        # 前进，过减速带
        if self.cc == 350:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 349 < self.cc <= 479:
            self.num = 5
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4)


        # 右旋转
        if self.cc == 480:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 479 < self.cc <= 519:
            self.circle_stop_90_right()
        
        # 绕大圈
        if self.cc == 520:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 519 < self.cc <= 569:
            self.num = 19
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4) 

        # 右旋转
        if self.cc == 570:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 569 < self.cc <= 609:
            self.circle_stop_90_right()

        
        # 前进
        if self.cc == 520:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 519 < self.cc <= 539:
            self.num = 5
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4) 

        # 暂停
        if self.cc == 540:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 539 < self.cc <= 549:
            self.num = 1
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4) 

        # 左旋转
        if self.cc == 550:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 549 < self.cc <= 574:
            self.circle_stop_90()


        # 前进
        if self.cc == 550:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 549 < self.cc <= 569:
            self.num = 19
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4) 


        # 独木桥，下楼梯，直行
        if self.cc == 570:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 569 < self.cc <= 789:
            self.num = 22
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4)

        # 暂停
        if self.cc == 790:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 789 < self.cc <= 809:
            self.num = 1
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4)

        # 左旋转
        if self.cc == 810:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 809 < self.cc <= 834:
            self.circle_stop_90()

    # red and green
        # lef_stop 
        if self.cc == 835:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 834 < self.cc <= 859:
            self.left_stop()

        # go_stop
        if self.cc == 860:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 859 < self.cc <= 884:
            self.go_stop_1()

        # right_stop
        if self.cc == 885:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 884 < self.cc <= 909:
            self.right_stop()

        # go_stop
        if self.cc == 910:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 9 < self.cc <= 934:
            self.go_stop()

        # left_stop
        if self.cc == 935:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 934 < self.cc <= 959:
            self.left_stop()

        # go_stop
        if self.cc == 960:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 959 < self.cc <= 984:
            self.go_stop_1()


        # right_stop
        if self.cc == 985:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 984 < self.cc <= 1009:
            self.right_stop()

        # go_stop
        if self.cc == 1010:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 1009 < self.cc <= 1034:
            self.go_stop_1()

        # 俯卧撑
        if self.cc == 1035:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 1034 < self.cc <= 1069:
            self.num = 27
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4)

        # 暂停
        if self.cc == 1070:
            self.msg.life_count += 1
            # print(self.msg.life_count)
        if 1069 < self.cc <= 1079:
            self.num = 1
            self.cc += 1
            self.update_and_publish()
            # print("num: " + str(self.num))
            # print("cc: " + str(self.cc))
            time.sleep(0.4)







        # # 完成序列后，可以在这里停止或者循环执行
        # self.running = False





    def left_stop(self):
        cc = 0
        while cc < 25 :
            if 0 <= cc <= 13:
                self.num = 7
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.2)          
            if 13 < cc <= 25:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.2)           
    


    def go_stop(self):
        cc = 0
        while cc < 25 :
            if 0 <= cc <= 13:
                self.num = 5
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.2)          
            if 13 < cc <= 25:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.2)      

    def go_stop_1(self):
        cc = 0
        while cc < 25 :
            if 0 <= cc <= 13:
                self.num = 5
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.4)         
            if 13 < cc <= 25:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.2)  


    def right_stop(self):
        cc = 0
        while cc < 25 :
            if 0 <= cc <= 13:
                self.num = 8
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.2)         
            if 13 < cc <= 25:
                self.num = 1
                self.cc += 1
                cc += 1
                # self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.2)   


    def circle_stop_180(self):
        cc = 0
        while cc < 40 :
            if 0 <= cc <= 35:
                self.num = 10
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.45)           
            if 35 < cc <= 40:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.4)     

    def circle_stop_90(self):
        cc = 0
        while cc < 40 :
            if 0 <= cc <= 17:
                self.num = 9
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.45)              
            if 17 < cc <= 25:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.4)  

    def circle_stop_90_right(self):
        cc = 0
        while cc < 40 :
            if 0 <= cc <= 17:
                self.num = 10
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.45)  #            
            if 17 < cc <= 25:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.4)  


    def right_jump(self):  
        cc = 0
        while cc < 25:
            if 0 <= cc <= 10:
                self.num = 13
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.1)  
            if 10 < cc <= 25:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.2)  


    def left_jump(self):
        cc = 0
        while cc < 25:
            if 0 <= cc <= 11:
                self.num = 11
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.1)   
            if 11 < cc <= 25:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.1)       





    def update_and_publish(self):
        self.msg.mode = self.steps["step"][self.num]["mode"]
        self.msg.gait_id = self.steps["step"][self.num]["gait_id"]
        self.msg.contact = self.steps["step"][self.num]["contact"]
        self.msg.value = self.steps["step"][self.num]["value"]
        self.msg.duration = self.steps["step"][self.num]["duration"]
        # if self.lastorder != self.num:
        #     self.msg.life_count += 1  # 每次发布new_order时增加life_count
        #     self.lastorder = self.num
        for i in range(3):
            self.msg.vel_des[i] = self.steps["step"][self.num]["vel_des"][i]
            self.msg.rpy_des[i] = self.steps["step"][self.num]["rpy_des"][i]
            self.msg.pos_des[i] = self.steps["step"][self.num]["pos_des"][i]
            self.msg.acc_des[i] = self.steps["step"][self.num]["acc_des"][i]
            self.msg.acc_des[i+3] = self.steps["step"][self.num]["acc_des"][i+3]
            self.msg.ctrl_point[i] = self.steps["step"][self.num]["ctrl_point"][i]
            self.msg.foot_pose[i] = self.steps["step"][self.num]["foot_pose"][i]
        for i in range(2):
            self.msg.step_height[i] = self.steps["step"][self.num]['step_height'][i]
        self.lc.publish("robot_control_cmd", self.msg.encode())

def main(args=None):
    rclpy.init(args=args)
    multi_topic_node = MultiTopicNode()
            # 启动控制线程
    ctrl_thread = threading.Thread(target=multi_topic_node.run)
    ctrl_thread.start()

    rclpy.spin(multi_topic_node)  # 启动事件循环

    multi_topic_node.running = False
    multi_topic_node.control_thread.join()  # 等待控制线程结束

    multi_topic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()