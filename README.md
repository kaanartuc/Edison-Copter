# Edison-Copter
Quadcopter with Intel Edison based flight controller


This is a quadcopter which is fully controlled by a flight controller system that is done with Intel Edison and necessary sensors. The quadcopter is controlled by a smart phone Blynk App.

Main Components
Intel Edison, 4in1 ESC 20A, 16 channel PWM Servo Driver(Since Edison's PWM pins aren't capable of sending low duty cycles), 10-DOF sensor, 4xBrushless motors

Quadcopter System and Mobile Application
Smart phone application gets user input data and sends this data to Intel Edison in flight controller through Wi-Fi. This data is sent to motors with help of PWM driver. When there is instability on the quadcopter system, gyro sensor involves and measures this error. We correct this measurement with PID calculations and send the error differences to motors to stabilize the system.

Working Principle of Edison-Copter
https://user-images.githubusercontent.com/16873223/30248135-052aa0ee-962b-11e7-825c-f705bfb6222d.jpg
