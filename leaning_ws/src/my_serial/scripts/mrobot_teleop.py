#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from my_serial.msg import arm
import sys, select, termios, tty

msg = """
Control mrobot!
---------------------------
Moving around:
         'w':(1), #前摆臂向上
        'z':(2),#前摆臂向下
        'a':(3),#后摆臂向上
        's':(4),#后摆臂向下
        'q':(10),#前摆臂回零
        'x':(11),#后摆臂回零

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

move_Bindings = {
        'w':(1,1), #前摆臂向上
        'z':(2,1),#前摆臂向下
        'a':(3,1),#后摆臂向上
        's':(4,1),#后摆臂向下
        'q':(10,1),#前摆臂回零
        'x':(11,1),#后摆臂回零
           }



# speedBindings={
#        'q':(1.1,1.1),
#         'z':(.9,.9),
#         'w':(1.1,1),
#         'x':(.9,1),
#         'e':(1,1.1),
#         'c':(1,.9),
#           }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

angle = 45
def vels(angle):
    return "currently:\tangle %s " % (angle)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('mrobot_teleop')
    pub = rospy.Publisher('/arm_vel', arm, queue_size=5)

    # x = 0 #摆动方向： 1向上 -1 向下 
    th = 0 #0表示前摆臂  1表示后摆臂
    # m=0    #3表示回零
    status = 0
    count = 0
    # acc = 0.1
    # target_angle = 0
    # control_angle = 0
    try:
        print msg
        print vels(angle)
        while(1):
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）
            if key in move_Bindings.keys():
                 th=move_Bindings[key][0] #表示取w或z的第一位

            # if key in mode_Bingdins.keys():
            #     m=mode_Bingdins[key][0]#表示取q的第一位 
            #     th=0
            #     x=-1
            #     m=0
            # if key in control_Bindings.keys():
            #     x = control_Bindings[key][0]#表示取a或s的第一位
            #     count = 0
            # 速度修改键
            # elif key in speedBindings.keys():
            #     angle = angle * speedBindings[key][0]  # 角度增加 1 度
            #     count = 0
            #  print vels(angle)
                # if (status == 14):
                #     print msg
                # status = (status + 1) % 15
            # 停止键
            elif key == ' ':
                th = 0    
            else:
                # count = count + 1
                # if count > 4:
                #     x = 0
                 if (key == '\x03'): #对应concel建
                    break 
            # 目标速度=速度值*方向值
            # target_angle = angle * x
            # # 速度限位，防止速度增减过快
            # if target_angle > control_angle:
            #     control_angle = min( target_angle, control_angle + 1 )
            # elif target_angle < control_angle:
            #     control_angle = max( target_angle, control_angle - 1 )
            # else:
            #     control_angle = target_angle

            # 创建并发布twist消息
            twist = arm()
            twist.addr=0 #表示取w或z的第一位
            twist.data=th #表示取a或s的第一位
            twist.mode=0 #表示取q的第一位 
            print ("addr=%d",th)
            # print ("data=%d",)
            # print ("mode=%d",m)
            # twist.linear.x = control_speed; 
            # twist.linear.y = 0; 
            # twist.linear.z = 0
            # twist.angular.x = 0; 
            # twist.angular.y = 0; 
            # twist.angular.z = control_turn
            pub.publish(twist)



    except:
        print e

    finally:
        twist = arm()
        twist.data = 0
        twist.mode = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
