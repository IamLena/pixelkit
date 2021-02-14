# Python 3 SDK

Check the [requirements](https://github.com/KanoComputing/community-sdk/wiki/Installation-Guides#python-3), [dependencies and how to install](https://github.com/KanoComputing/community-sdk/wiki/Installation-Guides#python-3-with-mu-editor) on the [Community SDK Wiki](https://github.com/KanoComputing/community-sdk/wiki).

There you can also find the [API Documentation](https://github.com/KanoComputing/community-sdk/wiki/Python-SDK-API-Documentation), [examples](https://github.com/KanoComputing/community-sdk/wiki/Documentation#python-3) and [insirational projects!](https://github.com/KanoComputing/community-sdk/wiki/Inspirational-Projects)

## Do you have a question, suggestion or a problem?

Don't worry: [Tell us about it](https://github.com/KanoComputing/community-sdk/issues)!


## My notes
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


By blacklisting it in /etc/modprobe.d/ SO MY PROBE FUNCTION WORKS WITH ARDRUINO

cd /sys/bus/usb/devices
ls -al
find definition as in dmesg, go to that folder

cd dev/char look for 180:0 - it is for usb to is the link to
dev/skel0 - is my char file to send data to device

"""""
exec 3<>/dev/skel0
stty -F /dev/skel0 9600 cs8 -cstopb -parenb
echo "hello" >$3
cat <$3
exec 3<&-  to close fd


"""
where I stopped -> write function is called but message is not printed on device
