from std_msgs.msg import Int32
import rospy

"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-


def CB(msg):
    print(msg.data*2)


# 1. node 이름 설정
rospy.init_node("wego_sub_node")
# 2. node 역할 설정
rospy.Subscriber("/counter", Int32, callback=CB)  # callback 함수
rospy.spin()
