#!/usr/bin/env python3

from itertools import zip_longest
import csv
import yaml
import rostopic
import rospy
import argparse

'''
    Author: Johan Fanas (@Johanfanas)

    This python script reads the config.yaml file and creates
    a callback for each ROS topic. The data is saved into a 
    csv file with the column name specified in the yaml file.
'''

class SUBHANDLER(object):

    def __init__(self, config_file, filename):

        '''
            TODO:
                Set if statement so it only starts when incoming data
                ERROR: Topic parameter 'data_class' is not initialized
        '''

        self.filename = filename

        self.variables = {}

        for topic in config_file:
            for name in config_file[topic]['column_names']:
                self.variables[config_file[topic]['column_names'][name]] = []

        for topic in config_file:
            message_type = rostopic.get_topic_class(config_file[topic]['name'])[0]

            rospy.Subscriber(config_file[topic]['name'], message_type,
                callback=self.callback_handler, callback_args=(config_file[topic]))

    def callback_handler(self, data, args):
        
        for col_name, param in zip(args['column_names'], args['parameters']):

            msg = data

            for s in args['parameters'][param].split('.'):
                msg = getattr(msg, s)

            self.variables[args['column_names'][col_name]].append( msg ) 

    def __del__(self):

        path = self.filename + '.csv'

        print('\033[92m' + "Writing csv file ", path)

        fields = self.variables.keys()

        # writing to csv file
        with open( path ,"w+") as f:
            # creating a csv writer object
            csvwriter = csv.writer(f)

            # writing the fields 
            csvwriter.writerow(fields)

            for values in zip_longest(*self.variables.values()):
                csvwriter.writerow(values)

parser = argparse.ArgumentParser(description='ROS to CSV')
parser.add_argument('-f', '--filename', default='output', help='Name of output csv file')

args = parser.parse_args()

try:

    with open('config.yaml', 'r') as file:
        params = yaml.safe_load(file)

    rospy.init_node('csv_node')
    obj = SUBHANDLER(params, args.filename)
    rospy.spin()
except KeyboardInterrupt:
    pass

'''
    Forces python to delete the object before the python interperter finishes
    to guarantee creating csv file
'''
del obj