# hik_publisher

## 启动(单个结点)

``ros2 run hik_publisher hik_publish_node``

## 话题

- 相机信息
  ``/camera_info``
- 图像信息
  ``/image_raw``

## 参数（文件位置：mosas_bringup/config）

- config/
  - camera_info : 相机信息，包括图像尺寸，外参内参，通过/camera_info发送
  - camera_param : 相机启动参数，可通过rqt_reconfigure动态调节
