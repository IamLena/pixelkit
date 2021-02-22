make to build
sudo insmod usb_skeleton.ko to install module to the kernel
sudo rmmod usb_skeleton to remove module from the kernel
dmesg to show kernel log

sudo rmmod ftdi_sio to remove the loaded driver
sudo modprobe ftdi_sio to restore it

ldd3
https://lwn.net/Kernel/LDD3/
http://dmilvdv.narod.ru/Translate/LDD3/Linux_Device_Drivers_3_ru.pdf
https://github.com/martinezjavier/ldd3/blob/master/usb/usb-skeleton.c



In Linux, the current user may not have access to serial ports and a "Permission Denied" error will appear. On most Linux distributions, the solution is to add the user to the dialout group with a command like sudo usermod -a -G dialout <USERNAME>. Check your Linux distribution's documentation for more information.

By blacklisting it in /etc/modprobe.d/ SO MY PROBE FUNCTION WORKS WITH ARDRUINO  cdc_acm

cd /sys/bus/usb/devices
ls -al
find definition as in dmesg, go to that folder

cd dev/char look for 180:0 - it is for usb to is the link to
dev/skel0 - is my char file to send data to device

"""""
exec 3<>/dev/skel0
<!-- stty -F /dev/skel0 9600 cs8 -cstopb -parenb --> not needed modified speed rate on the device to 115200
echo "hello" >$3
cat <$3
exec 3<&-  to close fd

"""

usb_driver usb_skeleton registers the device as 180:0 char device (/dev/skel0), sends data on write call, but device does not output anything

"""
init_serial_interface
https://github.com/dj0abr/ttyUSB-handler

"""
https://opensource.com/article/18/11/udev
https://stackoverflow.com/questions/28836712/how-do-you-get-a-struct-device-for-a-linux-character-device

"""
у меня вот тут есть вот такой вызов tty_register_device(tiny_tty_driver, i, NULL); И вместо NULL мне нужно передать указатель на struct device моего устройства. Вот как мне эту структуру вытащить, я не могу понять.

"""
https://elixir.bootlin.com/linux/latest/source/drivers/usb/class/cdc-acm.c

"""
unknown baud rate sometimes
default cdc_acm gets dissconnected somehow, it shows used by 0

""
look /proc/devices


usbmon
https://www.kernel.org/doc/html/latest/usb/usbmon.html
