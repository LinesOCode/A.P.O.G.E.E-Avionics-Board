# **A.P.O.G.E.E Avionics**

## Introduction
Hi! Welcome to my project, A.P.O.G.E.E Avionics! I Made this project as a challenge to myself, mainly because I want to soon be able to make a rocket that is entirely my own, design, build, and electronics, and this is a HUGE leap out to that, with the electronics outta the way, I can learn many, many things to come!

A.P.O.G.E.E stands for Avionics, Power, Orientation, Guidance, and Electronics Engine, it represents all of the things I want this project to grow into, and I hope it really does!

## How it works - 
### *Hardware*

This project is mainly based around the Raspberry Pi 2350 Series, i have designed this chip to work with the 2350B/2354B, as they have built in QSPI memory, and without it, you cannot upload the code. For the components, I wont go into extreme depth, but here is the most important things and what they are used for - 

The ***MPU*** - Raspberry Pi 2354B

The ***Accelerometer*** - LIS2DH12TR from STMicroelectronics

The ***Barometer*** - SPL07-003 from Goertek

The ***Gyroscope*** - A3G4250DTR from STMicroelectronics

The ***Inertial Mesurement Unit*** - LSM6DS3TR-C from STMicroelectronics

The ***Radio Transceiver*** - LoRa2 by Ai Thinker (Dont ask why the company has the word Ai in it, its one of the only ones that work)

### *Software*

***Code*** - I coded this project in C++, mainly because of the speed and reliability it has when in a task such as this, and as a fair declaration, I did use AI to code this, as I have no idea on how to use or code those, and it would take a substantial amount of time to learn it, and I WILL add to the code in the future, at least until it is reliable, but as of this being written, I don't have the PCB, nor do i even have funding, so I cant even test the code.

***The Filter*** - For this, I am using an Extended Kalman Filter based code, mainly because of its perks in the aerospace field, and just how good it is a weighting the different data between the components.
