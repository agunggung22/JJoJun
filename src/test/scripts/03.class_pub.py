"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

from std_msgs.msg import Int32
import rospy


class Class_pub:

    def __init__(self):
        # 1. node 이름 설정
        rospy.init_node("wego_pub_node")
        # 2. node 역할 설정 (필수 : 토픽 이름, 메세지 타입)
        self.pub = rospy.Publisher("/counter", Int32, queue_size=1)
        self.int_msg = Int32()
        # 3. rate 설정
        self.rate = rospy.Rate(1)  # 10hz : 1초에 10번
        self.num = 0

    def func(self):
        while not rospy.is_shutdown():
            self.num += 1
            self.int_msg.data = self.num
            self.pub.publish(self.int_msg)
            print(self.num)
            self.rate.sleep()  # 3-2. 주기 실행


def main():
    try:
        class_pub = Class_pub()
        class_pub.func()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
