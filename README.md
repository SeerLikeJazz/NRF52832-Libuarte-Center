# NRF52832-Libuarte-Center
主机接收，串口发送PC端  
![](/Image/USB_Dongle.jpg)  

## 更新记录

### 22.09.28
- 接收一个包220字节，0xBBAA+24x9+校验位+包序号=220字节
- 添加主机图片

### 22.09.26
- 固件配合自己做的USB Dongle主机硬件
- 修改 PCA10040.h文件，引脚
- FT232芯片，921600波特率正常

### 22.09.26
- 使用青风开发板开发
- 460800波特率，180x2x50=18k字节/s接收打印。CH340-921600波特率有问题

