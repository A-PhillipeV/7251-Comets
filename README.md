## Hi Programmers !!
This GitHub repository is the heart of 7251-Comets. This repo is for the 2022-2023 season POWER-PLAY  
7251 Handbook: https://docs.google.com/document/d/1x8hSlBrLIcQmxfHnqMxywp3bEHRAU8E4cPc_SarHzqw/edit?usp=sharing

## Using This Repo

__Disclaimer: If you use this repo by it's own, it will not work!!__  
Download the current FTC-Robot Controller SDK: https://github.com/FIRST-Tech-Challenge/FtcRobotController 
Go to the FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ folder and copypaste all of the given files in this rep. 
We are currrently using:
- FTC Dashboard
- RoadRunner
- EasyOpenCV
If you want to add these + any extra stuff, follow the instructions below !

## FTC Dashboard:
Follow guide on: https://acmerobotics.github.io/ftc-dashboard/gettingstarted  
To access FTC dash, type in:   
http://192.168.43.1:8080/dash

## EasyOpenCV:
Follow guide on: https://github.com/OpenFTC/EasyOpenCV  
For step 7, go to "Device File Explorer," and go through SD card.  

__For 2022-2023 season, we use april-codes for detection. Research more on EasyOpenCV!__  
For april-codes, follow guide on: https://github.com/OpenFTC/EOCV-AprilTag-Plugin

## Road Runner:
Follow instructions on: https://learnroadrunner.com/   

## Wireless Uploading
There are two ways to wirelessly upload code, and for the most part this is simply for pleasure without having to move around. If all fails, wire it up!  
__Easy Way:__ Download REV Hardware Client and connect to RC wifi

__Lengthy Way:__ Download the android platform_tools sdk:   
https://developer.android.com/studio/releases/platform-tools   
Click on the bar in platform_tools <img width="185" alt="platform_tool" src="https://user-images.githubusercontent.com/87776842/188527100-bea97838-59f2-486e-880b-9ba4bf24e8b9.png"> (where you see the path) and type **cmd**, and then type in the following:   

`abd tcpip 5555`    
`adb connect 192.168.43.1:5555 (5555 is the "dock" for connection)`   






