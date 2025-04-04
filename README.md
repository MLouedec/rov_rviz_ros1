This is a ROS Package I used during my Phd to display BlueRov rosbag on Rviz, version ROS1
I also have another repository for ROS2

The replay.launch read the rosbags of the rovs to display then on Rviz. There is also a kamlan filter to estimate the position of the ROV (propagation with input values, correction with USBL data)

The simu.launch can read the python simulation data in .json format and display it on rviz

The bag2csv convert the rosbag into a csv file to be read by Mathlab or Python
