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


'''读取指定引脚的信号，可选参数isSensor,判断是否启用红外传感器'''
def digitalRead(pin, isSensor = False):
    if isSensor:
        if pin in (0,1):
            return analog_read(pin) > 50
        else:
            return analog_read(pin) > 4000
    else:
        return digital_read(pin)

'''wifi连接时闪烁LED状态灯'''
def waitWifi():
    inet_line = ''
    LED = other.openLED
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
def pinSetUp(other):
    pin_mode(other.canOpenLED,OUTPUT)
    digital_write(other.canOpenLED,LOW)

    pin_mode(other.canCloseLED, OUTPUT)
    digital_write(other.canCloseLED, LOW)

    pin_mode(other.motorUp, OUTPUT)
    digital_write(other.motorUp, LOW)

    pin_mode(other.motorDown, OUTPUT)
    digital_write(other.motorDown, LOW)

    for floor in other.floors.values():
        if floor['button']:
            pin_mode(floor['button'], INPUT)
            pin_mode(floor['LED'], OUTPUT)
            digital_write(floor['LED'], LOW)

'''关闭所有楼层的LED灯'''
def closeAllFloorsLED(other):
    for floor in other.floors.values:
        if floor['LED']:
            digital_write(floor['LED'], LOW)

'''等待有人开门'''
def waitOpenDoor(other):
    digital_write(other.openLED, HIGH)      #打开指示灯，等待按下开门按钮
    print('等待开门')
    while digital_read(other.openDoor) != keyValidLevel:    #当有人按下开门按钮，结束循环
        pass
    digital_write(other.openLED, LOW)       #关闭指示灯
    print('检测到开门，开始检测人脸')

'''检测人脸'''
def detectFace():
    for i in range(10):
        __, img = Cam.read()
        if Fc.face_exist(img):              #判断是否有人脸
            faces = Fc.face_detect(img)     #读取所有人脸
            if not faces:                   #如果没有人脸，则返回False,否则返回人脸及楼层字典
                print("未检测到人")
                return False
            return dict.fromkeys(faces,[])

'''获得人脸以及对应的楼层'''
def getFaceAndFloors(unknowFacesFloors):        #参数为字典:人脸为键 空列表为值
    judgeFaces = []
    for knowFace in knowFacesFloors.keys():     #获取faceFloors中所有键
        for unknowFace in unknowFacesFloors.keys():
            print('开始比对人脸并尝试自动选择楼层')
            cmpResult = Fc.face_compare(knowFace,unknowFace)
            print('相似度: ' + str(cmpResult))
            if cmpResult > ConfidenceEdge:          #相似度大于阀值，则为已知人脸
                thePeopleFloors = knowFacesFloors[knowFace]        #获取该用户的楼层
                judgeFaces.append(knowFace)              #将已知人脸加入已判断出的人脸列表，以便后面对这些用户进行楼层整合
                del unknowFacesFloors[unknowFace]                   #将已知的人脸移出
                if len(thePeopleFloors) == 1:               #如果楼层只有一个，则加入楼层记录，并点亮对应楼层的LED灯
                    print("该用户去%d楼" % (thePeopleFloors[0]))
                    alreadyChooseFloors.add(thePeopleFloors[0])
                    digital_write( other.floors[thePeopleFloors[0]]['LED'], HIGH)

    print("自动选择楼层结束")
    digital_write(other.closeLED, LOW)              #? 为什么与下方的digital_write都是传输LOW
    print("等待检测")
    while digital_read(other.detect) != KeyValidLevel:      #循环遍历楼层按钮，直到按下‘检测按钮‘
        for floor in other.floors.keys():
            if floor in alreadyChooseFloors:               #如果该楼层已经在选择的楼层中,跳过
                continue
            if None == floors[floor]['button']:
                continue
            if KeyValidLevel == digital_read(floors[floor]['button']):  #未记录的楼层，且检测到按钮
                alreadyChooseFloors.add(floor)                      #将该楼层添加已选择楼层集合中
                digital_write(floors[floor]['LED'], HIGH)           #点亮该楼层灯
    digital_write(other.closeLED, LOW)
    print("选中楼层: " + str(alreadyChooseFloors))
    print("旧人脸有%d个" % len(judgeFaces))
    print("新人脸有%d个" % len(unKnowFacesFloors))                #剩余的未知人脸

    ##融合已判断出的人脸数据
    for judgeFace in judgeFaces:
        knowFacesFloors[judgeFace] = list( alreadyChooseFloors & set(knowFacesFloors[judgeFace]))   #将选择的楼层与旧数据做交集
    ##追加扩展新人脸的数据
    for unknowFace in unknowFacesFloors.keys():
        unknowFacesFloors[unknowFace] = alreadyChooseFloors            #将选择的楼层列表复制给新用户
    knowFacesFloors.update(unknowFacesFloors)                           #将新用户的数据加入旧用户中

'''电梯开始运行'''
def elevatorRun(elevator):
    ##楼层从低到高 前往目的地楼层
    for goToFloor in alreadyChooseFloors:
        print("正才前往%d楼" % (goToFloor))
        elevator.goto(goToFloor)
        digital_write(other.floors[goToFloor]['LED'], LOW)      #到达指定楼层后熄灭该楼层的灯,并停留2s
        sleep(2)
    print("全部完成，返回1楼")
    elevator.goto(1)

'''一些杂项(按钮，LED灯，电机，红外传感器，人脸阀值,有效电平)组成的类'''
class Other():
    openDoor = 12           #开门按钮
    detect = 13             #检测按钮
    # 各楼层对应的按钮，LED灯，红外传感器
    floors = {
        1: {'button': None, 'LED': None, 'sensor': 1},
        2: {'button': 5, 'LED': 10, 'sensor': 2},
        3: {'button': 4, 'LED': 9, 'sensor': 3},
        4: {'button': 3, 'LED': 8, 'sensor': 4},
        5: {'button': 2, 'LED': 7, 'sensor': 5},
    }
    openLED = 11         #指示灯
    closeLED = 6         #指示灯
    motorUp = 15            #电机上升
    motorDown = 14          #电机下降

'''定义电梯类，读入参数电机上升下降引脚。由楼层按钮，LED灯，红外传感器组成的floors字典。当前楼层'''
class Elevtor():
    def __init__(self,motorUp,motorDown,floors,currentFloor = 0):
        self.motorUp = motorUp
        self.motorDown = motorDown
        self.floors = floors
        self.currentFloor = currentFloor
        self.sensor = sensor

    def __up(self):
        digital_write(self.motorUp, HIGH)
        digital_write(self.motorUp, LOW)

    def __down(self):
        digital_write(self.motorUp, LOW)
        digital_write(self.motorUp, HIGH)

    def __stop(self):
        digital_write(self.motorUp, LOW)
        digital_write(self.motorUp, LOW)

    def goto(self,goToFloor):
        if goToFloor == self.currentFloor:
            return
        elif goToFloor > self.currentFloor:
            self.__up()
        elif goToFloor < self.currentFloor:
            self.__down()
        else:
            pass

        while digitalRead(floors[goToFloor]['sensor']):     #获取指定楼层的红外传感器信号
            pass
        sleep(3.5)
        self.currentFloor = goToFloor
        self.__stop()

KeyValidLevel = HIGH  # 按键有效的电平
ConfidenceEdge = 80     #人脸匹配阀值
Cam = cv2.VideoCapture
Fc = FaceppAPI()

if __name__ == '__main__':
    knowFacesFloors = {}  # 以人脸为键，楼层列表为值的字典
    other = Other()
    pinSetUp(other)
    elevator = Elevtor(other.motorUp,other.motorDown,other.floors,4)  #初始化当前楼层为4楼
    elevator.goto(1)        #前往0层
    waitWifi()

    while True:
        try:
            alreadyChooseFloors = set()             #已经选择的楼层列表(集合会对元素自动排序)
            closeAllFloorsLED(other)
            waitOpenDoor(other)
            unKnowFacesFloors = detectFace()  #如果有人脸则返回以人脸为键，空列表（用来存储每个人的楼层）为值的字典，否则返回False
            if False == faces:
                continue
            getFaceAndFloors(unKnowFacesFloors)
            elevatorRun(elevator)
        except ValueError:
            print("网络出现问题，等待重新连接")
            digital_write(other.openLED, LOW)
            digital_write(other.closeLED, LOW)
            waitWifi()
            continue

