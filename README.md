# Autolight

Autolight is a wall-mounted IoT product designed to manage porch lighting based on door movement and user activity. The sensing device can be placed on a shoe rack and doubles as an ornament. The display device attaches to the existing light switch and operates the light using a motor.

### 1. Sensing Device

The sensing device detects two types of actions: door movement and user activity. An ultrasonic sensor is used to accurately detect these actions.

![Image of sensing device](./img/sensingdevice.png)

### 2. Display Device

The display device is detachable from the existing light switch and controls the switch using a motor based on sensor readings. A tactile switch allows users to manually toggle the light’s state, while an LED on the device indicates the battery status.

![Image of display device](./img/displaydevice.png)

### 3. System Architecture

The sensing device and display device communicate through BLE.
