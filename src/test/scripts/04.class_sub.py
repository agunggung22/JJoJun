"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

from std_msgs.msg import Int32
import rospy


class Class_sub:
    def __init__(self):
        # 1. node 이름 설정
        rospy.init_node("wego_sub_node")
        # 2. node 역할 설정
        rospy.Subscriber("/counter", Int32, callback=self.CB)  # callback 함수

    def CB(self, msg):
        print(msg.data*2)  # 당일필드이므로 .data 사용


def main():
    try:
        class_sub = Class_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
