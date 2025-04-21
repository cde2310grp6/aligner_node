# aligner_node
This simple script is called by casualty_location/casualty_save via a ROS2 service, and returns a callback via ordinary ROS2 topics.
It operates on the following logic

1. receive service call from casualty_save
2. spin until heatsource is in the middle of the frame
3. move foward
4. stop when at a fixed threshold distance
5. send callback to casualty_save

It uses the ir_pub node's "ir_data" topic to gather information about the heatsource
and uses the LIDAR "/scan" topic to detect distance to heatsource
![image](https://github.com/user-attachments/assets/d06292df-09f2-4b2c-8ce5-f5bc7b125954)

