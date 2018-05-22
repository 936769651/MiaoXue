#!/usr/bin/python
#-*- coding:utf-8 -*-

'''
楼层从1楼开始,到5楼结束
'''

import time
from time import sleep
import cv2
from os import path, popen                                                          #os.popen从一个命令打开一个管道
from pcduino import *
from facepp_api import FaceppAPI


'''读取指定引脚的信号，可选参数is_sensor,判断是否启用红外传感器'''
def sensor_read(pin):
    if pin in (0,1):
        return analog_read(pin) > 50
    else:
        return analog_read(pin) > 4000
        
'''wifi连接时闪烁LED状态灯'''
def wait_wifi():
    inet_line = ''
    LED = PINID.open_LED
    while True:
        res = popen('ifconfig')
        lines = res.readlines()
        for i in range( len(lines) ):
            if lines[i].startswith('wlan'):
                inet_line = lines[i + 1]
                break
        if inet_line.strip().startswith('inet 地址'):
            break
        if digital_read(LED) == HIGH:
            digital_write(LED, LOW)
        else:
            digital_write(LED, HIGH)

'''引脚设置'''
def pin_setup():
    pin_mode(PINID.open_LED,OUTPUT)
    digital_write(PINID.open_LED,LOW)

    pin_mode(PINID.close_LED, OUTPUT)
    digital_write(PINID.close_LED, LOW)

    pin_mode(PINID.motor_up, OUTPUT)
    digital_write(PINID.motor_up, LOW)

    pin_mode(PINID.motor_down, OUTPUT)
    digital_write(PINID.motor_down, LOW)

    for floor in PINID.floors.values():
        if floor['button']:
            pin_mode(floor['button'], INPUT)
            pin_mode(floor['LED'], OUTPUT)
            digital_write(floor['LED'], LOW)

'''关闭所有楼层的LED灯'''
def close_all_floors_led():
    for floor in FLOORS.values():
        if floor['LED']:
            digital_write(floor['LED'], LOW)

'''等待有人开门'''
def wait_open_door():
    digital_write(PINID.open_LED, HIGH)                                     #打开指示灯，等待按下开门按钮
    print('等待开门')
    while digital_read(PINID.open_door) != KEY_VALID_LEVEL:                 #当有人按下开门按钮，结束循环
        pass
    digital_write(PINID.open_LED, LOW)                                      #关闭指示灯
    print('检测到开门，开始检测人脸')

'''检测人脸'''
def detect_face():
    img = None
    for i in range(10):                                                     #跳帧
        __, img = CAM.read()
    if FC.face_exist(img):                                                  #判断是否有人脸
        faces = FC.face_detect(img)                                         #读取所有人脸
        return faces    

'''获得人脸以及对应的楼层'''
def get_faces_and_floors(unknow_faces,know_faces_floors):                   #参数：未知人脸列表；已保存的由id,face,floors组成的字典
    judge_id_lst = []                                                       #匹配出的人脸id号列表
    for id, data in know_faces_floors:                                      #遍历已知用户(脸，楼层)的数据，进行匹配
        know_face = data['face']                                            #获取该用户的脸
        the_people_floors = data['floors']                                  #获取该用户的楼层
        for unknow_face in unknow_faces:
            print('开始比对人脸并尝试自动选择楼层')
            cmpResult = FC.face_compare(know_face,unknow_face)
            print('相似度: ' + str(cmpResult))

            if cmpResult > CONFIDENCE_EDGE:                                 #相似度大于阀值，则为已知人脸
                judge_id_lst.append(id)                                     #将匹配人脸id加入已匹配出的人脸id列表，以便后面对这些用户进行楼层数据整合
                unknow_faces.remove(unknow_face)                            #将已知的人脸移出
                if len(the_people_floors) == 1:                             #如果该用户的楼层只有一个，则直接加入楼层记录，并点亮对应楼层的LED灯
                    go_to_floor = the_people_floors[0]
                    print("该用户去%d楼" % go_to_floor)
                    already_choose_floors.add(go_to_floor)                  #将前往楼层加入已选择楼层集合
                    digital_write(FLOORS[go_to_floor]['LED'], HIGH)

    print("自动选择楼层结束")
    digital_write(PINID.close_LED, LOW)                                     #??# 为什么与下方的digital_write都是传输LOW

    print("等待检测")
    while digital_read(PINID.detect) != KEY_VALID_LEVEL:                    #循环遍历楼层按钮，直到按下‘检测按钮‘
        for floor in FLOORS.keys():
            if floor in already_choose_floors:                              #如果该楼层已经在选择的楼层中,跳过
                continue
            if KEY_VALID_LEVEL == digital_read(FLOORS[floor]['button']):    #未记录的楼层，且检测到按钮
                already_choose_floors.add(floor)                            #将该楼层添加已选择楼层集合中
                digital_write(FLOORS[floor]['LED'], HIGH)                   #点亮该楼层灯

    digital_write(PINID.close_LED, LOW)                                     #??#
    print("选中楼层: " + str(already_choose_floors))
    print("旧人脸有%d个" % len(judge_id_lst))
    print("新人脸有%d个" % len(unknow_faces))                                #剩余的未知人脸

    ##融合已判断出的人脸数据
    for id in judge_id_lst:
        know_faces_floors[id]['floors'] = list( already_choose_floors & set(know_faces_floors[id]['floors']))   #将选择的楼层集合与旧数据做交集

    ##遍历id,直到出现没有的id号，开始插入新用户数据
    new_id = 0
    while True:
        if new_id in know_faces_floors.keys():
            new_id += 1
        else:
            break
    ##追加新用户的数据
    for face in unknow_faces:
        know_faces_floors[new_id] = {'face':face, 'floors':list(already_choose_floors)}
        new_id += 1

'''电梯开始运行'''
def elevator_run(elevator,already_choose_floors):
    ##楼层从低到高 前往目的地楼层(集合已经自动排序)
    for go_to_floor in already_choose_floors:
        print("正才前往%d楼" % (go_to_floor))
        elevator.goto(go_to_floor)
        digital_write(FLOORS[go_to_floor]['LED'], LOW)                      #到达指定楼层后熄灭该楼层的灯,并停留2s
        sleep(2)
    print("全部完成，返回1楼")
    elevator.goto(1)


'''一些杂项(按钮，LED灯，电机，红外传感器，人脸阀值,有效电平)组成的类'''
class PinId():
    open_door = 12                                                          #开门按钮
    detect = 13                                                             #检测按钮
    open_LED = 11                                                           #指示灯
    close_LED = 6                                                           #指示灯
    motor_up = 15                                                           #电机上升
    motor_down = 14                                                         #电机下降

'''定义电梯类，读入参数电机上升下降引脚。由楼层按钮，LED灯，红外传感器组成的floors字典。当前楼层'''
class Elevtor():
    def __init__(self, current_floor = 0):
        self._current_floor = current_floor

    def __up(self):
        digital_write(PINID.motor_up, HIGH)
        digital_write(PINID.motor_down, LOW)

    def __down(self):
        digital_write(PINID.motor_up, LOW)
        digital_write(PINID.motor_down, HIGH)

    def __stop(self):
        digital_write(PINID.motor_up, LOW)
        digital_write(PINID.motor_down, LOW)

    def goto(self,go_to_floor):
        if go_to_floor == self._current_floor:
            return
        elif go_to_floor > self._current_floor:
            self.__up()
        elif go_to_floor < self._current_floor:
            self.__down()

        while sensor_read(FLOORS[go_to_floor]['sensor']):                   #获取指定楼层的红外传感器信号
            pass
        sleep(3.5)
        self._current_floor = go_to_floor
        self.__stop()



KEY_VALID_LEVEL = HIGH                                                      # 按键有效的电平
CONFIDENCE_EDGE = 80                                                        #人脸匹配阀值
CAM = cv2.VideoCapture
FC = FaceppAPI()                                                            #人脸识别接口
PINID = PinId()                                                             #一些引脚杂项（开门按钮，检测按钮，指示灯，电机）
FLOORS = {                                                                  #嵌套字典:各楼层的按键，LED,传感器所对应的引脚
        1: {'button': None, 'LED': None, 'sensor': 1},
        2: {'button': 5, 'LED': 10, 'sensor': 2},
        3: {'button': 4, 'LED': 9, 'sensor': 3},
        4: {'button': 3, 'LED': 8, 'sensor': 4},
        5: {'button': 2, 'LED': 7, 'sensor': 5},
    }

if __name__ == '__main__':
    know_faces_floors= {}                                                   #嵌套字典，存储用户id,脸,楼层   { id: {'face': face, 'floors': floors}}
    pin_setup()                                                             #初始化所有引脚
    elevator = Elevtor(4)                                                   #初始化当前楼层为4楼
    elevator.goto(1)                                                        #使电梯前往1层
    wait_wifi()                                                             #等待wifi

    while True:
        try:
            already_choose_floors = set()                                   #已经选择的楼层列表(集合会对元素自动排序)
            close_all_floors_led()                                          #关闭所有楼层的LED灯
            wait_open_door()                                                #等待有人开门
            unknow_faces = detect_face()                                    #返回检测到的人脸列表
            if len(unknow_faces) == 0:                                      #没有人脸则结束
                print("没有检测到人脸")
                continue
            get_faces_and_floors(unknow_faces, know_faces_floors)           #匹配人脸以及对应的楼层，并对旧数据进行更新
            elevator_run(elevator,already_choose_floors)                    #匹配,更新结束，电梯开始前往已选择的楼层
        except ValueError:
            print("网络出现问题，等待重新连接")
            digital_write(PINID.open_LED, LOW)
            digital_write(PINID.close_LED, LOW)
            wait_wifi()
            continue

