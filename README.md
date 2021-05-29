# Esp32Drone
This is A unique source code of Quadcopter written for Esp32.

In my AnjelDrone Repository you can see that i have used raspberry Pi, Arduino, Android app, to make a fully functional Quadcopter.

This project is more optimized and fast, in this project i have only used a ESP32 module and a transmitter.

# Why i use ESP32

Well we all know that Arduino has clock speed og 16mhz, so we can't perform tasks faster, drones need fast response time.
So , came up with the idea of ESP32 , ehich has clock speed of 240mhz.

# Problems faced
Arduino community is very vast , it is easy to debug code . Also most of the libraries are easily available.
But in case of ESP32 i have to come up with new ideas to implement functions like (PWM signal to ESC , interrupts) 

# What is important if you try to use this code
The most important thing required , if you wantt to use this code is SIMONK ESC , which has no buffering in input 
signal, means it need ESC with refresh rate more than 490hz and Motor grequency of 16KHZ.
You will need a logic level converter to read signals from reciever, because receiver signals are 5v logic , but ESP32
has 3.3v logic.

# Required Libraries
You have to use only these Libraries which i have uploaded, because i have changed a lot of code in libraries itself.
Using libraies from original source will redduce the drone response time.

# Connections - 
I will update the Connection diagram with ESP32 pins, SOOOOONNN.
