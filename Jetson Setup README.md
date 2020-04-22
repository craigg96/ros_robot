Instructions how to setup a fresh install of ubuntu 18.04 on a jetson nano

# Create folder directory 
    cd ~
    mkdir -p ros/robot/src
    # If possible add keys to ziva_keys folder now    

# Configure git and clone ziva repository

    git config --global push.default simple
    git config --global credential.helper store
    git config --global user.name "craigg96"
    git config --global user.email "craigg96@gmail.com"
    cd ~/ros/robot/src
    git clone https://github.com/craigg96/robot.git
    cd robot
    git checkout master

# Run setup scripts
    cd ~/ros/ziva/src/ziva/setup_scripts
    ./setup_udev_rules
    ./setup_jetson
    ./setup_jetson_ros

# If needing to setup additonal machine vision elements
Caution... Takes Ages!!!
    ./setup_jetson_openCV
    ./setup_jetson_interference
