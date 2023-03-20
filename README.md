ENPM 661- Planning for Autonomous Robots (Spring 23)

Project 3, Phase 1: Implementation A* algorithm for a mobile Robot

Team Members:

Name: Vipul Patel UID: 119395547

Name: Poojan Desai UID: 119455760

\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-

GitHub Links: https://github.com/vipulp16/ENPM661-Project-3_Phase-1

Drive link to output sample video:  https://drive.google.com/file/d/19YLIAQdG4kA8U5P354tskMTcjYstq_fo/view?usp=sharing

Code: - Code consists of Generating an Obstacle space, Possibe Movement in 5 Directions ( +60, +30, 0, -30, -60 degree), Using A* Search Algorithm, Back Tracking to find shortest path and Visualizing the Tracked Path. 
- The code has been delineated very clearly in the comments provided in the Code.

Dependencies: - python 3.11.0 (works for any python 3 version) 
- Python running IDE. (I used Visual Sudio Code to program the Code and Execute the Code) 
- Libraries to be install: numpy, time, math, cv2, sys, copy

Instructions to Run the Code:

To get the Output:
- Open the program file named \"a_star_vipul_poojan.py in any IDE. 
- Run the Program 

- In the Console, the program asks for: \-
- Robot clearence space \-
- Robot Radius \-
- Robot Step Size \-
- The x and y coordinates of Start and orientation of robot at start node. \-
- The x and y coordinates of Goal and orientation of robot at Goal node.  
Enter as prompted. Ex: Robot clearence space: 5, Robot Radius: 5 (As per requriement); Step size: 1; Start node(x and y): 50, 50; Initial orientation: 90; Goal node (x and y): 400, 20; Final orientation : 60. 
- The Output Plot with planned Path should be Visible.

Understanding the Output Plot 
- The Robot movable space is shown in Black Color 
- The Pixels in Red are the Obstacles. 
- The Pixels in Yellow is the Clearance Space. 
- The explored path is marked by white color. 
- The Planned Path is shown by Red Lines. 
- The start node is shown in goal node is shown in green color

Libraries Used: 
- numpy: Perform Operations on array 
- math : Perform mathematical operations 
- cv2 : Display the generated array as output 
- sys : Exit the code with a error message 
- copy : Copy the array 
- time : Calculate the time taken to find the path
