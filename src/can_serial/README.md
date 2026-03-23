# can_serial

can通信实现  
波特率1000000  
字节存储使用小端模式

## include
包含can和ros2头文件声明和can定义  
`CanSerialCore.hpp` can头文件声明  
`CanSerialCore.cpp` can函数定义  
`CanSerialNode.hpp` ros2 node头文件声明

## src
ros2 node代码放在此目录下的 `CanSerialNode.cpp` 文件中  

can通信的接收处理函数绑定必须在ros2创建完发布者之后，不然会出现 Segmentation Fault