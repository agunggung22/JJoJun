from std_msgs.msg import Int32
import rospy

"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

# 1. node 이름 설정
rospy.init_node("wego_pub_node")

# 2. node 역할 설정 (필수 : 토픽 이름, 메세지 타입)
pub = rospy.Publisher("/counter", Int32, queue_size=1)
int_msg = Int32()

# 3. rate 설정
rate = rospy.Rate(10)  # 10hz : 1초에 10번

num = 0
while not rospy.is_shutdown():
    num += 1
    int_msg.data = num
    pub.publish(int_msg)
    print(num)
    rate.sleep()  # 3-2. 주기 실행
