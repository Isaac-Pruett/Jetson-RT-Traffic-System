# Jetson-RT-Traffic-System
Caltrans Traffic Cam
By Marco Menashe, Micah Miller, Binh To, Jack Toyama

Continued from SURP 2025 Project for Cal Poly Slo. Real-time traffic detection with a Jetson Orin Nano.
By Isaac Pruett

# Features
1. MP4 and Cal Trans live cam video input
2. Vehicle outlines and path visualization in real-time
3. Base CSV creation with vehicle type, confidence level, vehicle ID, start, and end position
4. Cluster algorithm runs on base CSV input
5. Accurately assigns vehicles to a cluster and outputs CSV with base information plus cluster


commands to run everything off of a mp4
1. ros2 run rqt_image_view rqt_image_view
2. ros2 launch launch/deepstream_vid.py
3. ros2 launch launch/deepstream_cam.py (live cam run 1 and 3)

https://wzmedia.dot.ca.gov/D3/28_JCT267_JWO_KINGS_BEACH_PLA28_EB.stream/playlist.m3u8

#!/bin/bash 

# Terminal 1:  Image view    
gnome-terminal --bash -c "source install/setup.bash; ros2 run rqt_image_view rqt_image_view; exec bash"

sleep 2

# Terminal 2: Deepstream
gnome-terminal --bash -c "source install/setup.bash; ros2 launch launch/deepstream_vid.py; exec bash"

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
