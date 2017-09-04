#!/bin/bash

#Hello fellow software team members! 
#This script will set up your local machine with the most recent version of the
#git repository for trickfire robotics.

#everything will be written to a local directory called trickfire_robotics it
# will also create a local .gitignore for ease. We are only inlcuding souce
# code in our repository.

#To execute this script:
# (1) make sure you fill all of the prerequisites listed below
# (2) run the script from a bash terminal in the directory you want to create
#     the project

#PREREQUISITES
# -A linux operating system (we are running on Ubuntu 16.04 LTS)
# -Complete the ROS tutorials covering the build system
# -Have all neccessary ros software on kinetic kame 
#  (//wiki.ros.org/kinetic/Installation)
# -Have access rights to out git repository
# -Have git installed
# -Have git, and ros tools added to your path in bash 
#  (This would have probably been done in the tutorials for each tool)
# -Have SFML installed https://www.sfml-dev.org/

#Written by Collin collinsconway@gmail.com

#SCRIPT BODY

#clone the repository
GIT_URL="https://github.com/TrickFire-Robotics/rmc2017-recode.git"
GIT_NAME="trickfire_robotics"
echo "SCRIPT:   grabbing git repository at: $GIT_URL"
git clone $GIT_URL $GIT_NAME 
cd $GIT_NAME

#create a local git ignore
echo "SCRIPT:   creating local gitignore"
touch .gitignore
echo '.gitignore' >> .gitignore
echo '.catkin_workspace' >> .gitignore
echo 'build' >> .gitignore
echo 'devel' >> .gitignore

#build local packages
catkin_make
cd ..
