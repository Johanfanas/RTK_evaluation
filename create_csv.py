#!/usr/bin/env python3

import csv
import rostopic
import rospy

# def topic_handler(topic_name, message_type):

#     rospy.Subscriber( topic_name, message_type )

topics = int( input("Enter the number of topics you want to save into csv: ") )

names, col_names = [], []

for i in range(topics):

    names.append( input( "Enter name of topic No.{}: ".format(i) ) )
    col_names.append( input( "Column name in csv for topic No.{}: ".format(i) ) )
    # data = input( "Data to extract: " )
    # print("")

msg = rostopic.get_topic_class(names[0])[0]

rospy.init_node( 'node' )
m = rospy.wait_for_message( names[0], msg)
print(m.__slots__)
