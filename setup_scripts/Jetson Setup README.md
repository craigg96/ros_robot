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
    cd ~/ros/robot/src/robot/setup_scripts
    ./setup_jetson
    ./setup_jetson_ros


