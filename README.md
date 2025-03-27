[Work in Progress] This is the first iteration. Alot of the code is very messy and there are few to no code comments. The code is working however and I will do a second pass to tidy up everything.

To enter PATH mode you press A

To enter OBS mode you press S

To enter RAY mode you press D

Note: When the program first starts you are in RAY mode.

To print the arduino array press F


In PATH mode you can draw the path intended for the robot to follow. When you mouse button left click you place a point the robot is going to move towards. You can press Backspace to delete the most recently paced point. There is an upper limit of 40 points you can place.
<img src="https://github.com/user-attachments/assets/a21b63d0-021d-4e82-bb45-a9891bd9ea41" alt="Example of drawing a path with GUI" style="width:50%; height:auto;">

In OBS mode you can draw the obstacles in the environment. You click and drag to create the rectangles. You can press Backspace to delete the most recently placed rectangle. There is an upper limit of 40 rectangles you can place.
<img src="https://github.com/user-attachments/assets/b05fda2b-5667-4c75-bbde-d855416f4b6b" alt="Example of drawing rectangles with GUI" style="width:50%; height:auto;">

In RAY mode, the program calculates the sensor positions, directions and whether or not it collides with an obstacle then draws it to the screen. You can see the sensor positions (green circles), the sensor ray (red line) and the collision point (purple point). If that sensor position has no line then that sensor does not collide when the robot is at that position. The sensor positions can be altered by changing the sensorPos array in main.
<img src="https://github.com/user-attachments/assets/8d34341f-1e33-43b3-8dcb-25fbe45b407b" alt="Example of rays being calculated" style="width:50%; height:auto;">
