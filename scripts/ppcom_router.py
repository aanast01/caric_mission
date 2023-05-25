#!/usr/bin/env python

import rospy
from rotors_comm.msg import PPComTopology

# custom_topics = ['']
ppcomTopo = PPComTopology()

# def callback(data):
#     ppcomTopo = data
#     print(ppcomTopo.header)

def ppcom_callback(data):
    ppcomTopo = data
    # print(ppcomTopo.node_id)
    # print(ppcomTopo.node_role)
    # print(ppcomTopo.range)

if __name__ == '__main__':

    rospy.init_node('ppcom_router', anonymous=True)
    rospy.Subscriber("/gcs/ppcom_topology", PPComTopology, ppcom_callback)

    rospy.spin()
