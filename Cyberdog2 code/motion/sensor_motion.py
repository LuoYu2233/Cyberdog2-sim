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
        self.num = 1
        self.running1 = True
        self.running2 = False
        self.sensor_data_updated = False

        self.logger = self.get_logger()
        self.lidar_data = 0
        self.rgb_data = None
        self.lidar_data_0 = 0
        self.change = 1
        self.previous_num = 0
 

        self.lidar_subscription = self.create_subscription(
            Float32,
            'lidar_distance_90',
            self.lidar_callback,
            10)
        
        self.lidar_subscription = self.create_subscription(
            Float32,
            'lidar_distance_0',
            self.lidar_callback_0,
            10)
        self.rgb_subscription = self.create_subscription(
            String,
            'color_positions',
            self.rgb_callback,
            10)

        # 启动控制线程

        self.control_thread = threading.Thread(target=self.run)
        # self.control_thread = threading.Thread(target=self.run2)
        self.control_thread.start()


    def lidar_callback(self, msg):
        if msg.data > 0.2 :
            self.lidar_data = msg.data
        else :
            self.lidar_data = self.lidar_data
        self.logger.info(f'Lidar data: {self.lidar_data}')  # 打印激光雷达数据90度
        self.sensor_data_updated = True

    def lidar_callback_0(self, msg):
        if msg.data > 0.2 :
            self.lidar_data_0 = msg.data
        else :
            self.lidar_data_0 = self.lidar_data_0
        self.logger.info(f'Lidar data 0: {self.lidar_data_0}')  # 打印激光雷达数据0度
        self.sensor_data_updated = True


    def rgb_callback(self, msg):
        self.rgb_data = msg.data
        self.sensor_data_updated = True
        self.logger.info(f'rgb data: {self.rgb_data}')  # 打印rgb数据

    def update_num_based_on_sensors1(self):
        if  2.0 < self.lidar_data < 2.1 : 
            self.num = 1

        if 0.6 < self.lidar_data < 2.0 :
            self.num = 5

        if 0.4 < self.lidar_data < 0.6 and self.rgb_data == '0':
            self.right_jump()
            self.change += 1


    def update_num_based_on_sensors2(self):
        if 0.4 < self.lidar_data < 0.6 and self.rgb_data == '0':
            self.move_back()
            self.change += 1
        
    def update_num_based_on_sensors3(self):
        if self.rgb_data == '0':
            self.circle_stop_180()

        if 0.6 < self.lidar_data < 1.0 and self.rgb_data == '0' :
            self.num = 22
        
        if 0.4 < self.lidar_data < 0.6 and self.rgb_data == '0':
            self.circle_stop_90()

        if 0.3 < self.lidar_data and self.rgb_data == '2' : 
            self.num = 5

        if 0.2 < self.lidar_data <0.3 and self.rgb_data == '2' :
            self.circle_stop_90_right()
            self.change += 1

    def update_num_based_on_sensors5(self):
        if  self.rgb_data == '0':
            self.big_circle()
            self.change += 1

    def update_num_based_on_sensors6(self):
        if self.rgb_data == '0':
            self.circle_stop_90_right()
            self.change += 1

    def update_num_based_on_sensors7(self):
        if self.rgb_data == '0' and 0.6 < self.lidar_data :
            self.num = 5
        
        if self.rgb_data == '0' and 0.4 < self.lidar_data < 0.6 :
            self.circle_stop_90()

        if self.rgb_data == '4' :
            self.stop_and_move()
            self.change += 1


    def update_num_based_on_sensors8(self):
        if self.rgb_data == '5' :
            self.num = 22

        if 0.6 <self.lidar_data <4.0 and self.rgb_data == '0' :
            self.num = 22

        if 0.4 < self.lidar_data < 0.6 and self.rgb_data == '0':
            self.circle_stop_90()
            self.change += 1



    def update_num_based_on_sensors9(self):
        if 0.3 < self.lidar_data_0 < 1.0 and self.rgb_data == '7' :
            self.num = 7

        if self.lidar_data_0 > 1.0 and self.rgb_data == '7' :
            self.rgb_move()

        if self.lidar_data_0 > 1.0 and self.rgb_data == '8' :
            self.num = 8

        if  0.3 < self.lidar_data_0 < 0.4 and self.rgb_data == '8' :
            self.rgb_move()

        

            


    def run(self):
        
        while self.running1 :
            if self.sensor_data_updated:
                # 根据最新的传感器数据更新num
                if self.change == 1 :
                    self.update_num_based_on_sensors1()
                elif self.change == 2 :
                    self.update_num_based_on_sensors2()
                elif self.change == 3 :
                    self.update_num_based_on_sensors3()
                elif self.change == 4 :
                    self.update_num_based_on_sensors4()
                elif self.change == 5 :
                    self.update_num_based_on_sensors5()
                elif self.change == 6 :
                    self.update_num_based_on_sensors6()
                elif self.change == 7 :
                    self.update_num_based_on_sensors7()
                elif self.change == 8 :
                    self.update_num_based_on_sensors8()
                elif self.change == 9 :
                    self.update_num_based_on_sensors9()
                # 执行对应num的动作
                self.update_and_publish()
                # self.logger.info(f'Current count value: {self.count}')
                self.logger.info(f'change value: {self.change}')
                self.logger.info(f'Current num value: {self.num}')
                # 重置传感器数据更新标志
                self.sensor_data_updated = False
                # # 暂停一段时间，模拟动作执行的时间间隔
                time.sleep(0.2)




    def left_jump(self):
        cc = 0
        while cc < 25:
            if 0 <= cc <= 13:
                self.num = 11
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.1)  
            if 13 < cc <= 25:
                self.num = 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.2)     

    def right_jump(self):
        cc = 0
        while cc < 25:
            if 0 <= cc <= 13:
                self.num = 13
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.1)  
            if 13 < cc <= 25:
                self.num = 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.2)  

    def stop_and_move(self):
        cc = 0
        while cc < 35:
            if 0 <= cc <= 13:
                self.num = 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.2)  
            if 13 < cc <= 35:
                self.num = 22
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.2) 
   

    def left_jump_M(self):
        cc = 0
        while cc < 35:
            if 0 <= cc <= 13:
                self.num = 11
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.1)  
            if 13 < cc <= 25:
                self.num = 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.2)  
            if 25 < cc <= 35:
                self.num = 23
                cc += 1
                self.update_and_publish()
                time.sleep(0.1)

    def rgb_move(self):
        cc = 0
        while cc < 25:
            if 0 <= cc <= 13:
                self.num = 13
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.1)  
            if 13 < cc <= 25:
                self.num = 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.2)  

    def move_back(self):
        cc = 0
        while cc < 114:
            if 0 <= cc <= 101:
                self.num = 6
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.45)  
            if 101 < cc <= 114:
                self.num = 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(cc))
                time.sleep(0.2)  

    def circle_stop_180(self):
        cc = 0
        while cc < 40 :
            if 0 <= cc <= 35:
                self.num = 10
                self.cc += 1
                cc += 1
                self.update_and_publish()
                print("num: " + str(self.num))
                print("cc: " + str(self.cc))
                print("real_cc: " + str(cc))
                time.sleep(0.45)  #           
            if 35 < cc <= 40:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                print("num: " + str(self.num))
                print("cc: " + str(self.cc))
                print("real_cc: " + str(cc))
                time.sleep(0.4)  #   

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
        
    def big_circle(self):
        cc = 0
        while cc < 45 :
            if 0 <= cc <= 40:
                self.num = 10
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.4)  #            
            if 40 < cc <= 45:
                self.num = 1
                self.cc += 1
                cc += 1
                self.update_and_publish()
                # print("num: " + str(self.num))
                # print("cc: " + str(self.cc))
                # print("real_cc: " + str(cc))
                time.sleep(0.4)  




    def update_and_publish(self):
        self.msg.mode = self.steps["step"][self.num]["mode"]
        self.msg.gait_id = self.steps["step"][self.num]["gait_id"]
        self.msg.contact = self.steps["step"][self.num]["contact"]
        self.msg.value = self.steps["step"][self.num]["value"]
        self.msg.duration = self.steps["step"][self.num]["duration"]
        # self.msg.life_count += 1  # 每次发布时增加life_count
        if self.previous_num != self.num:  # 检查num值是否变化
            self.num = self.previouu_num  # 更新self.num
            self.msg.life_count += 1  # num值变化时，life_count加1
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
        # for i in range(300): #60s Heat beat, maintain the heartbeat when life count is not updated
        #     self.lc.publish("robot_control_cmd",self.msg.encode())
        #     time.sleep( 0.2 ) 

def main(args=None):
    rclpy.init(args=args)
    multi_topic_node = MultiTopicNode()

    rclpy.spin(multi_topic_node)  # 启动事件循环

    multi_topic_node.running = False
    multi_topic_node.control_thread.join()  # 等待控制线程结束

    multi_topic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()