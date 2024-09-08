# pgm_map_creator
将gazebo中的地图转为栅格地图。
安装时需要先编译一遍通过，然后取消注释：
![image](https://github.com/user-attachments/assets/a7f39472-4c21-4e7f-b954-176b710ee73c)
这是msgs文件夹下的cmakelists.txt。
这样就能正常使用了。
使用方法：
首先要在world文件的末尾添加插件：
![image](https://github.com/user-attachments/assets/a4c1a8c0-6056-49b9-935b-1a5dfdaad0e9)
然后启动gazebo:
roslaunch gazebo_ros empty_world.launch world_name:=world文件
最后启动转换：
roslaunch pgm_map_creator request_publisher.launch
然后就会在指定目录下生成对应的pgm和yaml文件
