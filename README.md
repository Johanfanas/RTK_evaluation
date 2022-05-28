# RTK_evaluation

This repository contains a script for some GPS evaluation using a covariance confidence ellipse and reported information from the sensor. It also possesses a script that reads a config.yaml file containing name of ROS topics, ROS message type and columns names for a csv file. The create_csv.py creates a ROS subscriber for each topic read and then stores the data into a csv file with the specified column name.

## Utilization

Use the create_csv.py script to collect the GPS data from a rosbag by modifying the config.yaml file. First launch roscore, start a bag and immediately stop it so the topic names are stored in buffer. Run the create_csv.py and then the rosbag to record the data.

## GPS evaluation

Go to the main function in the plot_csv.py in order to make the appropriate changes for your convinience. Change the csv file, plot numbering, etc. If anyting is done successfully, you should see an output like below:

![alt text](https://github.com/Johanfanas/RTK_evaluation/blob/main/images/confidence_ellipse.png?raw=true)
