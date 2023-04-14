# Robot Kinematics
This repository contains code I developed for my thesis: Exploring Robot
Kinematics: an Engineering Approach. Development went through a few stages which
involve organization changes and additions in technology. Each of the
subdirectories found here has their own README with information specific to that
stage in development.

## Learning Dynamixel
The first stage in development was when I was learning to use the DynamixelSDK
to communicate with the WidowX200 robot arm. I made very primitive simple single
motor programs to learn how to use each of the functions provided as an API to
the Dynamixel Protocol version 2.0. These simple programs worked up to more
complex programs that control multiple motors making up an entire movement for
the robot arm. After implementing the simple programs I made a wrapper for the
dynamixel sdk to simplify function calls and reduce code volume in the main
programs.

## Dynamixel Helper
This stage technically happened between the simple and complex programs from
when I was learning to use the SDK. Calling functions from the SDK ended up very
repetitive and cluttered my files when doing error checking. The Dynamixel
Helper is a shared library that I include in my other files to easily send
instructions to and handle status responses from the Dynamixel motors.

# Demos
Include videos and pictures from all stages of progression.
