This is a very motivational README.md for Altar's Audi Autonomous Driving Cup work.

##### Woohoo. Let's get to work.

# Getting set-up:

- Get ubuntu machine connected (can bypass ban through hub and manually setting ipv4 addresses)
- Change MAC address to registered one
- Pull ADTF project files from Gitlab
- Get ADTF from car (nomachine, in /opt) and set license
- Install CLion, open user project and set ADTF_DIR = /opt/ADTF/ and QT_DIR = (check opt gcc path) in cmake settings -> environment variables
- Run sudo apt-get install graphviz (might exist)
- Run sudo apt-get install libopenexr-dev
- Install opencv 3.4 from source with contrib modules included, instructions are given in the link: https://www.audi-autonomous-driving-cup.com/forums/topic/compiling-opencv/#post-4983 (do not git clone the extra-contrib repository, download the 3.4.1 release instead)
- Install pylon (64bit)

For assistance with set-up, talk to Lukas or Aimilios

# Git dev process

For development:
- DO NOT CREATE GRAPHS/OPEN PROJECTS ON YOUR COMPUTER (only on the car, at test time)
- Create branches for each feature to be developed (in src/aadcUser)
- Create graphs on the spot, on the car.

To test on car:
- Git pull, + optionally checkout to test your branch.
- Create graph/session and run on car

# Coding Conventions

Rule ONE: Be consistent!

Read this first: https://github.com/Microsoft/AirSim/blob/master/docs/coding_guidelines.md

Extra: https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Rp-direct

https://google.github.io/styleguide/cppguide.html

# How to be Real-time compliant

From ADTF3 guide:

- Do not allocate heap or stack memory during runtime (for code executed in realtime context). Do so during the initialisation phase.
- Do not perform blocking operations. These include file access, socket access as well as the use of mutexes and semaphores.
- Do not use string operations which usually require dynamic memory allocation and access is always non-deterministically due to the dynamic length of strings.
- Avoid the use of STL containers whose sorting algorithms are not deterministically and whose allocators use dynamically allocated heap memory by default.
- Avoid to use the unoptimised compilers Debug mode.
- Avoid any kind of user-input during runtime. It is advised to prefer a remote shell over a physical keyboard.

# How to get the car to start
- Make sure ADTF session is running
- Motor button pressed
- One long press and then short press on motor button inside the car
- Switch on autonomous
- If still not working restart arduinos and your program

# Notes:

- For opencv pkg_config issue: https://stackoverflow.com/questions/17831136/error-in-pkg-config with path from opt/opencv/lib
- On the car, the audi folder must be in /home, or else the build scripts won't work
- If camera does not open, check if system is not windows... some sessions are buggy and default to windowns which you can't change.
- Add to launch script: export QT_QUICK_BACKEND=software (to have qt run on cpu and not gpu)
- Build type to be used: RelWithDebInfo for consistency (not Debug)
- ADTF Training 1: https://www.audi-autonomous-driving-cup.com/wp-content/files/20180619_AADC_ADTF_1_Training_record.mp4
- Lukas Note to self: never disconnect the monitor cable when the computer is on, it will not reconnect.