#!/usr/bin/python
#-*- coding:utf-8 -*-

'''
楼层从1楼开始,到5楼结束
'''

import time
from time import sleep
import cv2
from os import path, popen #os.popen从一个命令打开一个管道
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
    for floor in PINID.floors.values:
        if floor['LED']:
            digital_write(floor['LED'], LOW)

'''等待有人开门'''
def wait_open_door():
    digital_write(PINID.open_LED, HIGH)      #打开指示灯，等待按下开门按钮
    print('等待开门')
    while digital_read(PINID.open_door) != KEYVALIDLEVEL:    #当有人按下开门按钮，结束循环
        pass
    digital_write(PINID.open_LED, LOW)       #关闭指示灯
    print('检测到开门，开始检测人脸')

'''检测人脸'''
def detect_face():
    for i in range(10):
        __, img = CAM.read()
    if FC.face_exist(img):              #判断是否有人脸
        faces = FC.face_detect(img)     #读取所有人脸
        return faces    

'''获得人脸以及对应的楼层'''
def get_faces_and_floors(unknow_faces):        #参数为字典:人脸为键 空列表为值
    judge_faces = []
    for know_face in know_faces_floors.keys():     #获取faceFloors中所有人脸
        for unknow_face in unknow_faces:
            print('开始比对人脸并尝试自动选择楼层')
            cmpResult = FC.face_compare(know_face,unknow_face)
            print('相似度: ' + str(cmpResult))
            if cmpResult > CONFIDENCEEDGE:          #相似度大于阀值，则为已知人脸
                the_people_floors = know_faces_floors[know_face]        #获取该用户的楼层
                judge_faces.append(know_face)              #将已知人脸加入已判断出的人脸列表，以便后面对这些用户进行楼层整合
                unknow_faces.remove(unknow_face)                  #将已知的人脸移出
                if len(the_people_floors) == 1:               #如果楼层只有一个，则加入楼层记录，并点亮对应楼层的LED灯
                    go_to_floor = the_people_floors[0]
                    print("该用户去%d楼" % go_to_floor)
                    already_choose_floors.add(go_to_floor)
                    digital_write( PINID.floors[go_to_floor]['LED'], HIGH)

    print("自动选择楼层结束")
    digital_write(PINID.close_LED, LOW)              #? 为什么与下方的digital_write都是传输LOW
    print("等待检测")
    while digital_read(PINID.detect) != KEYVALIDLEVEL:      #循环遍历楼层按钮，直到按下‘检测按钮‘
        for floor in PINID.floors.keys():
            if floor in already_choose_floors:               #如果该楼层已经在选择的楼层中,跳过
                continue
            if None == floors[floor]['button']:
                continue
            if KEYVALIDLEVEL == digital_read(floors[floor]['button']):  #未记录的楼层，且检测到按钮
                already_choose_floors.add(floor)                      #将该楼层添加已选择楼层集合中
                digital_write(floors[floor]['LED'], HIGH)           #点亮该楼层灯
    digital_write(PINID.close_LED, LOW)
    print("选中楼层: " + str(already_choose_floors))
    print("旧人脸有%d个" % len(judge_faces))
    print("新人脸有%d个" % len(unknow_faces))                #剩余的未知人脸

    ##融合已判断出的人脸数据
    for judge_face in judge_faces:
        know_faces_floors[judge_face] = list( already_choose_floors & set(know_faces_floors[judge_face]))   #将选择的楼层与旧数据做交集
    ##追加扩展新人脸的数据
    unknow_faces_floors = dict.fromkeys(unknow_faces, list(already_choose_floors))           #将选择的楼层列表复制给每个新用户
    know_faces_floors.update(unknow_faces_floors)                           #将新用户的数据加入旧用户中

'''电梯开始运行'''
def elevator_run(elevator,already_choose_floors):
    ##楼层从低到高 前往目的地楼层
    for go_to_floor in already_choose_floors:
        print("正才前往%d楼" % (go_to_floor))
        elevator.goto(go_to_floor)
        digital_write(PINID.floors[go_to_floor]['LED'], LOW)      #到达指定楼层后熄灭该楼层的灯,并停留2s
        sleep(2)
    print("全部完成，返回1楼")
    elevator.goto(1)

'''一些杂项(按钮，LED灯，电机，红外传感器，人脸阀值,有效电平)组成的类'''
class PinId():
    open_door = 12           #开门按钮
    detect = 13             #检测按钮
    # 各楼层对应的按钮，LED灯，红外传感器
    floors = {
        1: {'button': None, 'LED': None, 'sensor': 1},
        2: {'button': 5, 'LED': 10, 'sensor': 2},
        3: {'button': 4, 'LED': 9, 'sensor': 3},
        4: {'button': 3, 'LED': 8, 'sensor': 4},
        5: {'button': 2, 'LED': 7, 'sensor': 5},
    }
    open_LED = 11         #指示灯
    close_LED = 6         #指示灯
    motor_up = 15            #电机上升
    motor_down = 14          #电机下降

'''定义电梯类，读入参数电机上升下降引脚。由楼层按钮，LED灯，红外传感器组成的floors字典。当前楼层'''
class Elevtor():
    def __init__(self, current_floor = 0):
        self.__motor_up = PIN.motor_up
        self.__motor_down = PIN.motor_down
        self.__floors = PIN.floors
        self.__current_floor = PIN.current_floor

    def __up(self):
        digital_write(self.__motor_up, HIGH)
        digital_write(self.__motor_down, LOW)

    def __down(self):
        digital_write(self.__motor_up, LOW)
        digital_write(self.__motor_down, HIGH)

    def __stop(self):
        digital_write(self.__motor_up, LOW)
        digital_write(self.__motor_down, LOW)

    def goto(self,go_to_floor):
        if go_to_floor == self.__current_floor:
            return
        elif go_to_floor > self.__current_floor:
            self.__up()
        elif go_to_floor < self.__current_floor:
            self.__down()

        while sensor_read(__floors[go_to_floor]['sensor']):     #获取指定楼层的红外传感器信号
            pass
        sleep(3.5)
        self.__current_floor = go_to_floor
        self.__stop()

KEYVALIDLEVEL = HIGH  # 按键有效的电平
CONFIDENCEEDGE = 80     #人脸匹配阀值
CAM = cv2.VideoCapture
FC = FaceppAPI()
PINID = PinId()

if __name__ == '__main__':
    know_faces_floors= {}  # 以人脸为键，楼层列表为值的字典
    pin_setup()
    elevator = Elevtor(4)  #初始化当前楼层为4楼
    elevator.goto(1)        #前往1层
    wait_wifi()

    while True:
        try:
            already_choose_floors = set()             #已经选择的楼层列表(集合会对元素自动排序)
            close_all_floors_led()
            wait_open_door(PINID)
            unknow_faces = detect_face()  #返回检测到的人脸列表
            if len(unknow_faces) == 0:
                print("没有检测到人脸")
                continue
            get_faces_and_floors(unknow_faces)
            elevator_run(elevator,already_choose_floors)
        except ValueError:
            print("网络出现问题，等待重新连接")
            digital_write(PINID.open_LED, LOW)
            digital_write(PINID.close_LED, LOW)
            wait_wifi()
            continue

