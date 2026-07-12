cd /home/robosub/guppy
source install/setup.bash

until ping -c1 192.168.194.95 &>/dev/null; do
    echo "Waiting for DVL..."
    sleep 1
done

echo ----------ATTEMPTING TO START GUPPY----------
ros2 launch guppy hw.xml
