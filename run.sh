#!/bin/bash

# Attivare l'ambiente virtuale
source ~/sentinel_venv/bin/activate

# Caricare l'ambiente ROS 2
source /opt/ros/jazzy/setup.bash

# Pulizia dei file di build precedenti
rm -rf build/ install/ log/

# Aggiornare il PYTHONPATH per includere i pacchetti ROS 2 e l'ambiente virtuale
export PYTHONPATH=~/sentinel_venv/lib/python3.*/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH

# Costruire il progetto
colcon build --symlink-install || exit 1

# Sorgente dell'ambiente ROS 2 locale
source install/setup.bash

# Avvio del nodo
ros2 launch robot_controller robot_control_launch.py

