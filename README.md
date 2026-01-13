Pseudo Code:
Specific Packages (in order)
block_detection_aruco.py
get_perspective_warping_with_aruco.py
kinematic_functions.py
main_pipeline.py

Camera and Perception Pipeline	
Calibrate the camera using the 4 point selection at the four corners (blocks) on the table
Define the table-frame coordinates with those points
Use getPerspectiveTransform(image_points, table_points) to get a perspective matrix

Convert ROS image to OpenCV
Use getAruco() to detect AruCo tags and get the ID’s of the blocks
Find (u,v) in Pixels for AruCo tags

Call image_frame_to_table_frame()
	Convert [u,v,1] to an array x
Multiply the perspective matrix with x to get x_1
Normalize the new x_1 to get (X,Y) in the table-frame
Convert to string and enter them into the two value parameters
Return table-frame coordinates.

Move the arm into home position

Block Positioning Pipeline: 
	Normal Moving Block:
for each detected AruCo tag:
get aruco_id and (X, Y) position
Convert X, Y from mm to meters

if aruco_id == 100:
block ← Yellow
final_pose ← position 1
else if aruco_id == 150:
block ← Red
final_pose ← position 2
else if aruco_id == 200:
block ← Blue
final_pose ← position 3

initial_pose = [x, y, fixed_z, fixed_yaw]
Stacking Block:
	for i in range len(block_array):
	_, (x,y) = block_array[i]
Convert (x,y) from mm to meters

set the initial & final positions of whichever block is selected as:
initial_pose = [x, y, fixed z, hblock, yawnormal to plane]
final_pose = [xset, yset, ((i + 1) * hblock) - (0.01 * i), yawnormal to plane]

return marker centers in table frame

In both cases, call move_block(initial_pose, final_pose)

Gripper Vacuum:
Create message
Set destination to the current joint position
Set velocity and acceleration
Set io_0 to the toggle gripper state
Update internal gripper state
Publish command

Inverse Kinematics:
	Change world-frame gripper position to base frame

Compute wrist center position
Solve angles
return the joint angle vector

Block Movement:
Compute joint angles above the block with inverse kinematics function
# NOTE: the above word means setting the z coordinate at a sufficient height
Compute joint angles at initial_pose
move_arm(above_initial_pose) 
move_arm(initial_pose)
Turn the vacuum on
move_arm(above_initial_pose)

Compute joint angles at final_pose
move_arm(above_final_pose)
move_arm(final_pose)
Turn the vacuum off
move_arm(above_final_pose)


Math for Camera Frame

	Due to the angle and lens of the camera, it views the plane where the blocks are as a trapezoid rather than a rectangular plane. That is what the W accounts for. W accounts for as the plane recedes into the distance, it stretches while the nearer side of the plane is compressed. By using the perspective matrix and dividing by W we are able to translate the skewed view of the camera into a rectangular world frame. This is how we can take the coordinates of the AruCo markers and translate it into the robot frame.

Camera Code:
for i in range(len(ids)):
           (u, v) = marker_centers[i]
           x = np.array([u, v, 1])
           x_1 = perspective_matrix @ x
           w = x_1[2]
           X = x_1[0]/w
           Y = x_1[1]/w
           marker_centers_in_table_frame[i] = (X,Y)
           value1 = "" + str(X)
           value2 = "" + str(Y)

Videos: 

Video (Moving)
https://drive.google.com/file/d/1anGoMAa75H5UNUav1xSw50_WAiU5DUoS/view?usp=sharing

Video (Stacking)
https://drive.google.com/file/d/1LADMD5lRQMgZj-iVaFUt7IhhlmOxSwE0/view?usp=sharing

