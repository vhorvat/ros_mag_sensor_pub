# ros_mag_sensor_pub

## How to create symbolic links using serials and udev rules

udev is a part of the Linux subsystem responsible for dynamically creating and managing devices in the /dev/ directory. Every time you connect a device 
(such as a USB device, keyboard, mouse, etc.), udev creates the corresponding device (e.g., /dev/ttyUSB0) and manages it. When a new device is connected, 
udev can apply specific rules that determine how the device will be named, what permissions it will have, and what additional steps will be taken 
(such as creating symbolic links). 

These rules can be defined based on device characteristics such as the serial number, vendor ID (idVendor), product ID (idProduct), and others.

## 1) Get the serial number of the desired sensor
```
udevadm info -a -n /dev/ttyACM0 | grep '{serial}' | head -n1
```

## 2) Create udev rules file
```
sudo nano /etc/udev/rules.d/99-arduino.rules
```

## 3) Create rules
```
SUBSYSTEM=="tty", ATTRS{serial}=="123xyz", SYMLINK+="senzor1"
SUBSYSTEM=="tty", ATTRS{serial}=="456xyz", SYMLINK+="senzor2"
```

## 4) Reload rules
```
sudo udevadm control --reload-rules
```

Made at FER with ❤️
