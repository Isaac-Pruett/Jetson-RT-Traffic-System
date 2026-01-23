# Jetson-RT-Traffic-System
SURP 2025 Project for Cal Poly Slo. Real-time traffic detection with a Jetson Orin Nano.
test push to the repo from 2nd board



commands to run everything off of a mp4
1. ros2 run rqt_image_view rqt_image_view
2. ros2 launch launch/deepstream_vid.py
3. ros2 launch launch/deepstream_cam.py (live cam run 1 and 3)



#!/bin/bash 

# Terminal 1:  Image view    
gnome-terminal --bash - c "source install/setup.bash; ros2 run rqt_image_view rqt_image_view; exec bash"

sleep 2

# Terminal 2: Deepstream
gnome-terminal --bash - c "source install/setup.bash; ros2 launch launch/deepstream_vid.py; exec bash"

sleep 2

# Terminal 3: CSV Subcriber and Publisher
gnome-terminal --bash -c "source install/setup.bash; ros2 run inputCSVSubcriber inputCSVSubcriber; exec bash"

sleep 2

# Terminal 4: Cluster Algorithm
gnome-terminal --bash -c "source install/setup.bash; ros2 run cluster_algorithm cluster_algorithm; exec bash"


First:
    touch run_all.sh // This creates a shell script file called run_all

gnome-terminal -- bash -c "COMMAND; exec bash"
This means:
    - Open a new terminal
    - Run COMMAND
    - Keep the terminal open after the command runs

If we didn’t put exec bash, the terminal would close immediately.

Second
gedit run_all.sh
add:
    #!/bin/bash // Tells Linux to run this file using Bash (terminal language). Won't work without this
    
    gnome-terminal --bash - c "source install/setup.bash; ros2 run rqt_image_view rqt_image_view; exec bash"

    sleep 2

    gnome-terminal --bash - c "source install/setup.bash; ros2 launch launch/deepstream_vid.py; exec bash"

    sleep 2

    gnome-terminal --bash -c "source install/setup.bash; ros2 run inputCSVSubcriber inputCSVSubcriber; exec bash"

    sleep 2

    gnome-terminal --bash -c "source install/setup.bash; ros2 run cluster_algorithm cluster_algorithm; exec bash"

Then make it executable, give it permission to run:
chmod +x run_all.sh

Run it:
    ./run_all.sh