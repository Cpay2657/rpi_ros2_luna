readme
docker exec -it cs1 bash
ros2 topic pub /hello std_msgs/msg/String "data: hi"
M
ros2 run rpi_pubsub listener 

