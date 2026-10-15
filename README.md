# **A.P.O.G.E.E Avionics**

## Introduction
Hi! Welcome to my project, A.P.O.G.E.E Avionics! I Made this project as a challenge to myself, mainly because I want to soon be able to make a rocket that is entirely my own, design, build, and electronics, and this is a HUGE leap out to that, with the electronics outta the way, I can learn many, many things to come!

A.P.O.G.E.E stands for Avionics, Power, Orientation, Guidance, and Electronics Engine, it represents all of the things I want this project to grow into, and I hope it really does!

## How it works - 
### *Hardware*

This project is mainly based around the Raspberry Pi 2350 Series, i have designed this chip to work with the 2350B/2354B, as they have built in QSPI memory, and without it, you cannot upload the code. For the components, I wont go into extreme depth, but here is the most important things and what they are used for - 

The ***MPU*** - Raspberry Pi 2354B (Main Processing Unit, The Brains of the Avionics Board)

The ***Accelerometer*** - LIS2DH12TR from STMicroelectronics (Sees how fast the rocket changes speed, and uses that to find the speed of the rocket)

The ***Barometer*** - SPL07-003 from Goertek (Finds the hight of the rocket somehow idk man electronics are weird)

The ***Gyroscope*** - A3G4250DTR from STMicroelectronics (Finds the orientation of the rocket, so that we know if we are aiming straight at the ground or up at the sky)

The ***Inertial Mesurement Unit*** - LSM6DS3TR-C from STMicroelectronics (Backup for the Accelerometer and Gyroscope, and also includes a temperature sensor, so I can collect data too!)

The ***Radio Transceiver*** - LoRa2 by Ai Thinker (Dont ask why the company has the word Ai in it, its one of the only ones that work, It is used to send the telemetry data back to my computer)

### *Software*

***Code*** - I coded this project in C++, mainly because of the speed and reliability it has when in a task such as this, and as a fair declaration, I did use AI to code this, as I have no idea on how to use or code those, and it would take a substantial amount of time to learn it, and I WILL add to the code in the future, at least until it is reliable, but as of this being written, I don't have the PCB, nor do I even have funding, so I cant even test the code.

***The Filter*** - For this, I am using an Extended Kalman Filter based code, mainly because of its perks in the aerospace field, and just how good it is a weighting the different data between the components. This hasn't been tuned yet, but when I get funding, and when the board has been built, I should be able to introduce it to my components.

***The Website*** - I plan on adding a website that reads the telemetry data from the board and shows all of the data in real time, along with graphs and models of the direction/vectors affecting it to show the forces acting upon the rocket. I haven't made the website yet, but when I get to tinkering around with the actual hardware, it'll be no. 1 on my priorities list


## ***The Schemaics/Wiring Stuff***

 
#### For the Wiring/Silkscreen,

 
<img width="881" height="447" alt="Screenshot 2026-10-14 214444" src="https://github.com/user-attachments/assets/7f47c32a-2e68-4fcb-bed9-5e73da0097c6" />

This is what the final wire layout looks like, it has 4 layers, with this stack up (front to back)  Active Components -> Ground Layer -> Power/Auxiliary Layer -> Passive Components. I added a few references to spacecraft, like including what was printed on the Golden Record, or like the "Made on Earth by Humans from the Tesla Roadster *SpaceX* sent up"
 


  
#### For the Schematic,

 
<img width="893" height="675" alt="Screenshot 2026-10-14 214631" src="https://github.com/user-attachments/assets/faa6c435-2a32-4a0a-863d-07bc14c327a3" />

This is what the final schematic is, it has lots of connections, so I did my best to keep it tidy by only using GlobalTags to connect pins/pads. There isnt much to explain here, unless I wanted to explain every connection, but at that point, I might as well just publish a book on it, I already gave you hardware specs, so yeah.




#### Board Looks

I gotta make the board look like it's *mine* yk? So I added some *personal touches* to make it something I'm super proud to look at, so here's what it'll look like!

<img width="668" height="677" alt="Screenshot 2026-10-14 220647" src="https://github.com/user-attachments/assets/ba664771-06a2-4940-b751-87d880aac36a" />
<img width="277" height="550" alt="Screenshot 2026-10-14 220719" src="https://github.com/user-attachments/assets/63ec0e57-d301-41c7-98c2-dfbc932c6200" />
<img width="301" height="322" alt="Screenshot 2026-10-14 220738" src="https://github.com/user-attachments/assets/03971d0e-f2a5-437d-9770-02926c0408bd" />

I want the board to be white because I think it just looks much cleaner, and makes it different from other boards, making it feel uniquely *mine* (and yours if you wanna build it!) All of the code and hardware specs will be here and free to use, as long as you give a hats-off to this project if you used it!
