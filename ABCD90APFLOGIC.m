% Clear workspace and load image
clc; clear all;
startup_rvc;
%host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
rtdeport = 30003;
rtde = rtde(host, rtdeport);
vacuumPort = 63352;
vacuum = vacuum(host, vacuumPort);
home = [-588.53, -133.30, 493.70, 2.221, 2.221, 0]; % Modified to that the arm isn't visible on the camera
rtde.movej(home);
pause(5); % This is for getting the camera to take picture of the board
%rtde.actualJointPositions()
%rtde.actualPosePositions()
% Load image (replace with actual path)
% I = imread("F:\Project 2 - Release-20250803\example_4.jpg");
% I = I(:, 300:1700, :); % Only with example 3 for some reason
%For camera
%webcamlist
cam = webcam(2);
I = snapshot(cam);
%I = I(:, 300:1700, :);
figure(1); imshow(I); title('Original Image');
in = input("What Part Do You want? (Not Case Sensitive) : ", 's');
if strcmpi(in, 'A')
 item = 'small';
 PartA(I, item);
elseif strcmpi(in, 'B')
 item = 'small';
 [warpedImg, start_pos, goal_pos, obstacles] = PartA(I, item);
 PartB(warpedImg, start_pos, goal_pos, obstacles);
elseif strcmpi(in, 'C')
 item = 'small';
 [warpedImg, start_pos, goal_pos, obstacles, tilt_angle] = PartA(I, item);
 PartC(warpedImg, start_pos, goal_pos, obstacles, rtde, tilt_angle,vacuum);
 %PartC(warpedImg, start_pos, goal_pos, obstacles, rtde, tilt_angle);
elseif strcmpi(in, 'D')
 % item = 'small';
 % [warpedImg, start_pos, goal_pos, obstacles, tilt_angle] = PartA(I, item);
 % PartC(warpedImg, start_pos, goal_pos, obstacles, rtde, tilt_angle,vacuum);
 % pause(1);
 item = 'big';
 [warpedImg, start_pos, goal_pos, obstacles, tilt_angle] = PartA(I, item);
 PartD(warpedImg, start_pos, goal_pos, obstacles, rtde, tilt_angle, vacuum);
elseif strcmpi(in, 'E')
 item = 'small';
 [warpedImg, start_pos, goal_pos, obstacles, tilt_angle] = PartA(I, item);
 PartE(warpedImg, start_pos, goal_pos, obstacles, robot, tilt_angle); % FIXED: Updated reference
end
function [warpedImg, start_pos, goal_pos, obstacles, tilt_angle] = PartA(I, item)
 % First perform perspective transformation
 warpedImg = perspectiveTransform(I);
 
 % Then detect objects in the transformed image
 [warpedImg, start_pos, goal_pos, obstacles, tilt_angle] = detectObjects(warpedImg, item);
end
function path = PartB(warpedImg, start_pos, goal_pos, obstacles)
 % Create a new figure for APF visualization
 figure(4);
 imshow(warpedImg);
 hold on;
 
 % Define the grid for the potential field
 gridSpacing = 10; % mm
 x_min = 0; % Left edge
 x_max = 760; % Right edge
 y_min = 0; % Bottom edge
 y_max = 580; % Top edge
 [X, Y] = meshgrid(x_min:gridSpacing:x_max, y_max:-gridSpacing:y_min);
 
 % Calculate potential field (corrected to point toward goal)
 [Fx, Fy] = calculatePotentialField(X, Y, goal_pos, obstacles);
 
 % Display vector field
 quiver(X, Y, Fx, Fy, 0.5, 'y');
 
 % Plan and plot path
 path = planPath(start_pos, goal_pos, X, Y, Fx, Fy);
 plot(path(:,1), path(:,2), 'Color', [0.5 0 0.5], 'LineWidth', 2);
 
 hold off;
end
function PartC(warpedImg, start_pos, goal_pos, obstacles, robot, tilt_angle, vacuum) % FIXED: Parameter name
 % ---- Robot constants ----
 home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0];
 lift_height = 12; % mm above table for picking/placing
 approach_height = 6; % mm for approach/depart
 
 % ---- Move to home ----
 robot.movej(home); % FIXED: Updated reference
 
 % ---- Get path from PartB ----
 pathXY = PartB(warpedImg, start_pos, goal_pos, obstacles);
 
 % ---- Convert to robot coordinates ----
 robot_start = imageToRobot(start_pos);
 robot_goal = imageToRobot(goal_pos);
 robot_path = imageToRobot(pathXY);
 
 % ---- Build trajectory ----
 path = [];
 
 % 1. Move above start position
 path = [path; robot_start(1), robot_start(2), approach_height, home(4:6)];
 
 % 2. Move down to pick height
 path = [path; robot_start(1), robot_start(2), lift_height, home(4:6)];
 
 % 3. Activate vacuum (COMMENTED OUT for simulation)
 vacuum.grip();
 pause(1);
 % 4. Lift item
 path = [path; robot_start(1), robot_start(2), lift_height, home(4:6)];
 
 %5. Follow the robot path
 for i = 1:size(robot_path, 1)
 x = robot_path(i, 1);
 y = robot_path(i, 2);
 path = [path; x, y, lift_height, home(4:6)];
 end
 
 % 6. Move above goal
 path = [path; robot_goal(1), robot_goal(2), lift_height, home(4:6)];
 
 % 7. Move down to place height
 path = [path; robot_goal(1), robot_goal(2), approach_height, home(4:6)];
 
 % ---- Convert path to RTDE format ----
 rtde_path = [];
 v = 0.5; a = 1.2; blend = 0.005;
 for i = 1:size(path,1)
 point = [path(i,:), a, v, 0, blend];
 rtde_path = cat(1, rtde_path, point);
 end
 
 poses = robot.movej(rtde_path); % FIXED: Updated reference
 robot.drawPath(poses); % FIXED: Updated reference
 % ---- Rotate Wrist 3 to proper orientation ----
 current_joints = robot.actualJointPositions(); % FIXED: Updated reference
 current_joints_deg = rad2deg(current_joints);
 new_joints_deg = current_joints_deg;
 new_joints_deg(6) = new_joints_deg(6) + tilt_angle;
 new_joints_rad = deg2rad(new_joints_deg);
 robot.movej(new_joints_rad, 'joint'); % FIXED: Updated reference
 pause(2);
 % 8. Deactivate vacuum (COMMENTED OUT for simulation)
 vacuum.release();
 pause(0.2);
 
 % 9. Lift up from goal
 final_lift = [robot_goal(1), robot_goal(2), lift_height, home(4:6)];
 robot.movej(final_lift); % FIXED: Updated reference
 % ---- Plot path for verification ----
 figure;
 plot3(path(:,1), path(:,2), path(:,3), 'm-o', 'LineWidth', 2);
 xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
 grid on; title('Planned 3D Path for Small Item');
 % ---- Return to home ----
 robot.movej(home); % FIXED: Updated reference
end
function PartD(warpedImg, start_pos, goal_pos, obstacles, robot, tilt_angle, vacuum) % FIXED: Parameter name
 % ---- Robot constants ----
 home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0];
 lift_height = 12; % mm above table for picking/placing
 approach_height = 6; % FIXED: Changed from -6 to 6 for safety
 intermediate_height = 20; % mm for safe rotation height
 
 fprintf('\n=== PART D: Pick and Place with 90° Rotation ===\n');
 fprintf('Start position: [%.2f, %.2f]\n', start_pos);
 fprintf('Goal position: [%.2f, %.2f]\n', goal_pos);
 fprintf('Original tilt angle: %.2f degrees\n', tilt_angle);
 fprintf('Number of obstacles: %d\n', size(obstacles, 1));
 
 % ---- Move to home ----
 fprintf('\n--- Moving to home position ---\n');
 try
 robot.movej(home); % FIXED: Updated reference
 fprintf('✓ Successfully moved to home\n');
 catch ME
 fprintf('✗ Failed to move to home: %s\n', ME.message);
 rethrow(ME);
 end
 
 % ---- Get path from PartB ----
 fprintf('\n--- Planning path with PartB ---\n');
 try
 pathXY = PartB(warpedImg, start_pos, goal_pos, obstacles);
 fprintf('✓ Path planning successful, %d waypoints generated\n', size(pathXY, 1));
 catch ME
 fprintf('✗ Path planning failed: %s\n', ME.message);
 rethrow(ME);
 end
 
 % ---- Convert to robot coordinates ----
 fprintf('\n--- Converting coordinates ---\n');
 try
 robot_start = imageToRobot(start_pos);
 robot_goal = imageToRobot(goal_pos);
 robot_path = imageToRobot(pathXY);
 
 fprintf('Robot start: [%.2f, %.2f]\n', robot_start);
 fprintf('Robot goal: [%.2f, %.2f]\n', robot_goal);
 fprintf('✓ Coordinate conversion successful\n');
 catch ME
 fprintf('✗ Coordinate conversion failed: %s\n', ME.message);
 rethrow(ME);
 end
 
 % ---- Build trajectory ----
 fprintf('\n--- Building pick sequence trajectory ---\n');
 path = [];
 
 % 1. Move above start position
 path = [path; robot_start(1), robot_start(2), approach_height, home(4:6)];
 fprintf('Waypoint 1 (above start): [%.2f, %.2f, %.2f]\n', ...
 robot_start(1), robot_start(2), approach_height);
 
 % 2. Move down to pick height
 path = [path; robot_start(1), robot_start(2), lift_height, home(4:6)];
 fprintf('Waypoint 2 (pick height): [%.2f, %.2f, %.2f]\n', ...
 robot_start(1), robot_start(2), lift_height);
 
 % 3. Activate vacuum (COMMENTED OUT for simulation)
 fprintf('\n--- Activating vacuum gripper (SIMULATION) ---\n');
 vacuum.grip();
 pause(1);
 fprintf('✓ Vacuum activated (simulated)\n');
 % 4. Lift item to intermediate height for safe rotation
 path = [path; robot_start(1), robot_start(2), intermediate_height, home(4:6)];
 fprintf('Waypoint 3 (intermediate height): [%.2f, %.2f, %.2f]\n', ...
 robot_start(1), robot_start(2), intermediate_height);
 
 % ---- Convert path to RTDE format and execute pick sequence ----
 fprintf('\n--- Executing pick sequence ---\n');
 rtde_path = [];
 v = 0.5; a = 1.2; blend = 0.005;
 for i = 1:size(path,1)
 point = [path(i,:), a, v, 0, blend];
 rtde_path = cat(1, rtde_path, point);
 end
 
 try
 poses = robot.movej(rtde_path); % FIXED: Updated reference
 robot.drawPath(poses); % FIXED: Updated reference
 fprintf('✓ Pick sequence executed successfully\n');
 catch ME
 fprintf('✗ Pick sequence failed: %s\n', ME.message);
 rethrow(ME);
 end
 
 % 5. Rotate by 90 degrees while holding the item
 fprintf('\n--- Performing 90° rotation for obstacle avoidance ---\n');
 try
 current_joints = robot.actualJointPositions(); % FIXED: Updated reference
 current_joints_deg = rad2deg(current_joints);
 fprintf('Current joint angles (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
 current_joints_deg);
 
 new_joints_deg = current_joints_deg;
 
 % Add 90 degrees to current wrist rotation, then account for original tilt
 rotation_90_deg = 90; % Rotate 90 degrees for obstacle avoidance
 new_joints_deg(6) = new_joints_deg(6) + rotation_90_deg - tilt_angle; % Subtract original tilt
 
 % Store the total rotation applied for later correction
 total_rotation_applied = rotation_90_deg - tilt_angle;
 fprintf('Applying rotation: 90° obstacle avoidance - %.2f° original tilt = %.2f° total\n', ...
 tilt_angle, total_rotation_applied);
 fprintf('New joint 6 angle: %.2f° (was %.2f°)\n', ...
 new_joints_deg(6), current_joints_deg(6));
 
 new_joints_rad = deg2rad(new_joints_deg);
 robot.movej(new_joints_rad, 'joint'); % FIXED: Updated reference
 fprintf('✓ 90° rotation completed\n');
 pause(2);
 
 catch ME
 fprintf('✗ 90° rotation failed: %s\n', ME.message);
 rethrow(ME);
 end
 
 % 6. Follow the robot path at intermediate height
 fprintf('\n--- Following path to goal ---\n');
 path_transport = [];
 for i = 1:size(robot_path, 1)
 x = robot_path(i, 1);
 y = robot_path(i, 2);
 path_transport = [path_transport; x, y, intermediate_height, home(4:6)];
 end
 
 % 7. Move above goal at intermediate height
 path_transport = [path_transport; robot_goal(1), robot_goal(2), intermediate_height, home(4:6)];
 
 % Execute transport path
 try
 rtde_path_transport = [];
 for i = 1:size(path_transport,1)
 point = [path_transport(i,:), a, v, 0, blend];
 rtde_path_transport = cat(1, rtde_path_transport, point);
 end
 poses = robot.movej(rtde_path_transport); % FIXED: Updated reference
 robot.drawPath(poses); % FIXED: Updated reference
 fprintf('✓ Transport path executed successfully\n');
 catch ME
 fprintf('✗ Transport path failed: %s\n', ME.message);
 rethrow(ME);
 end
 
 % 8. Rotate to final orientation
 fprintf('\n--- Applying final orientation ---\n');
 try
 current_joints = robot.actualJointPositions(); % FIXED: Updated reference
 current_joints_deg = rad2deg(current_joints);
 new_joints_deg = current_joints_deg;
 
 % Remove the 90 degree rotation and apply the goal tilt angle
 new_joints_deg(6) = new_joints_deg(6) - rotation_90_deg + tilt_angle;
 % U CHECK THE SINGS R CORREC TOR NOT OTHERWSIE OPPSOITE 
 fprintf('Correcting orientation: removing 90° rotation + applying %.2f° goal tilt\n', tilt_angle);
 
 new_joints_rad = deg2rad(new_joints_deg);
 robot.movej(new_joints_rad, 'joint'); % FIXED: Updated reference
 fprintf('✓ Final orientation applied\n');
 pause(2);
 catch ME
 fprintf('✗ Final orientation failed: %s\n', ME.message);
 rethrow(ME);
 end
 
 % 9. Move down to place height
 fprintf('\n--- Placing item ---\n');
 place_path = [robot_goal(1), robot_goal(2), approach_height, home(4:6)];
 try
 robot.movej(place_path); % FIXED: Updated reference
 fprintf('✓ Moved to place height\n');
 catch ME
 fprintf('✗ Move to place height failed: %s\n', ME.message);
 rethrow(ME);
 end
 % 10. Deactivate vacuum (COMMENTED OUT for simulation)
 fprintf('\n--- Releasing vacuum (SIMULATION) ---\n');
 vacuum.release();
 pause(0.2);
 fprintf('✓ Vacuum released (simulated)\n');
 
 % 11. Lift up from goal
 final_lift = [robot_goal(1), robot_goal(2), lift_height, home(4:6)];
 try
 robot.movej(final_lift); % FIXED: Updated reference
 fprintf('✓ Lifted from goal position\n');
 catch ME
 fprintf('✗ Lift from goal failed: %s\n', ME.message);
 rethrow(ME);
 end
 % 12. Return to home
 fprintf('\n--- Returning home ---\n');
 try
 robot.movej(home); % FIXED: Updated reference
 fprintf('✓ Successfully returned to home\n');
 catch ME
 fprintf('✗ Return to home failed: %s\n', ME.message);
 rethrow(ME);
 end
 
 fprintf('\n=== PART D COMPLETED SUCCESSFULLY ===\n');
end
function PartE(warpedImg, start_pos, goal_pos, obstacles, robot, tilt_angle) % FIXED: Parameter name
 % ---- Robot constants ----
 home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0];
 lift_height = 12; % mm above table for picking/placing
 approach_height = 6; % FIXED: Changed from -6 to 6 for safety
 
 fprintf('\n=== PART E: Inverse Kinematics Pick and Place ===\n');
 fprintf('Start position: [%.2f, %.2f]\n', start_pos);
 fprintf('Goal position: [%.2f, %.2f]\n', goal_pos);
 fprintf('Tilt angle: %.2f degrees\n', tilt_angle);
 fprintf('Number of obstacles: %d\n', size(obstacles, 1));
 
 % ---- Move to home ----
 fprintf('\n--- Moving to home position ---\n');
 try
 robot.movej(home); % FIXED: Updated reference
 fprintf('✓ Successfully moved to home\n');
 catch ME
 fprintf('✗ Failed to move to home: %s\n', ME.message);
 rethrow(ME);
 end
 
 % ---- Get path from PartB ----
 fprintf('\n--- Planning path with PartB ---\n');
 try
 pathXY = PartB(warpedImg, start_pos, goal_pos, obstacles);
 fprintf('✓ Path planning successful, %d waypoints generated\n', size(pathXY, 1));
 catch ME
 fprintf('✗ Path planning failed: %s\n', ME.message);
 rethrow(ME);
 end
 
 % ---- Convert to robot coordinates ----
 fprintf('\n--- Converting coordinates ---\n');
 try
 robot_start = imageToRobot(start_pos);
 robot_goal = imageToRobot(goal_pos);
 robot_path = imageToRobot(pathXY);
 
 fprintf('Robot start: [%.2f, %.2f]\n', robot_start);
 fprintf('Robot goal: [%.2f, %.2f]\n', robot_goal);
 fprintf('✓ Coordinate conversion successful\n');
 catch ME
 fprintf('✗ Coordinate conversion failed: %s\n', ME.message);
 rethrow(ME);
 end
 
 % ---- Build Cartesian path ----
 fprintf('\n--- Building Cartesian trajectory ---\n');
 path = [];
 
 % 1. Move above start position
 path = [path; robot_start(1), robot_start(2), approach_height, home(4:6)];
 
 % 2. Move down to pick height 
 path = [path; robot_start(1), robot_start(2), lift_height, home(4:6)];
 
 % 3. Lift item
 path = [path; robot_start(1), robot_start(2), lift_height, home(4:6)];
 
 % 4. Follow the robot path
 for i = 1:size(robot_path, 1)
 x = robot_path(i, 1);
 y = robot_path(i, 2);
 path = [path; x, y, lift_height, home(4:6)];
 end
 
 % 5. Move above goal
 path = [path; robot_goal(1), robot_goal(2), lift_height, home(4:6)];
 
 % 6. Move down to place height
 path = [path; robot_goal(1), robot_goal(2), approach_height, home(4:6)];
 
 fprintf('Total waypoints: %d\n', size(path,1));
 
 % ---- Inverse Kinematics: Cartesian path -> joint path ----
 fprintf('\n--- Computing Inverse Kinematics ---\n');
 q0 = robot.actualJointPositions(); % FIXED: Updated reference
 fprintf('Initial joint configuration (rad): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', q0);
 
 successful_waypoints = 0;
 failed_waypoints = 0;
 
 for i = 1:size(path,1)
 % Extract Cartesian pose 
 x = path(i,1);
 y = path(i,2);
 z = path(i,3);
 rx = path(i,4);
 ry = path(i,5);
 rz = path(i,6);
 
 fprintf('\n--- Waypoint %d/%d ---\n', i, size(path,1));
 fprintf('Target Cartesian: [%.2f, %.2f, %.2f, %.3f, %.3f, %.3f]\n', ...
 x, y, z, rx, ry, rz);
 % Compute IK with previous solution as initial guess
 try
 q_solution = inverse_kinematics(x, y, z, rx, ry, rz, q0);
 q0 = q_solution; % Update for next iteration
 successful_waypoints = successful_waypoints + 1;
 
 fprintf('✓ IK solution found\n');
 fprintf('Joint solution (rad): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', q_solution);
 
 % Send joint commands to robot
 try
 robot.movej(q_solution, 'joint'); % FIXED: Updated reference
 fprintf('✓ Robot moved to waypoint %d\n', i);
 pause(0.1); % Small pause between moves
 catch ME
 fprintf('✗ Robot movement failed for waypoint %d: %s\n', i, ME.message);
 failed_waypoints = failed_waypoints + 1;
 end
 
 catch ME
 fprintf('✗ IK failed for waypoint %d: %s\n', i, ME.message);
 failed_waypoints = failed_waypoints + 1;
 
 % Try to continue with Cartesian move as fallback
 try
 fprintf('Attempting Cartesian fallback move...\n');
 cartesian_pose = [x, y, z, rx, ry, rz];
 robot.movej(cartesian_pose); % FIXED: Updated reference
 fprintf('✓ Cartesian fallback successful for waypoint %d\n', i);
 catch ME2
 fprintf('✗ Cartesian fallback also failed: %s\n', ME2.message);
 continue;
 end
 end
 end
 
 fprintf('\n--- IK Summary ---\n');
 fprintf('Successful waypoints: %d/%d\n', successful_waypoints, size(path,1));
 fprintf('Failed waypoints: %d/%d\n', failed_waypoints, size(path,1));
 if size(path,1) > 0
 fprintf('Success rate: %.1f%%\n', (successful_waypoints/size(path,1))*100);
 end
 
 % ---- Rotate Wrist 3 to proper orientation ----
 fprintf('\n--- Applying final wrist orientation ---\n');
 try
 current_joints = robot.actualJointPositions(); % FIXED: Updated reference
 current_joints_deg = rad2deg(current_joints);
 
 new_joints_deg = current_joints_deg;
 new_joints_deg(6) = new_joints_deg(6) + tilt_angle;
 
 fprintf('Applying tilt angle: %.2f degrees\n', tilt_angle);
 
 new_joints_rad = deg2rad(new_joints_deg);
 robot.movej(new_joints_rad, 'joint'); % FIXED: Updated reference
 fprintf('✓ Final orientation applied\n');
 pause(2);
 
 catch ME
 fprintf('✗ Final orientation failed: %s\n', ME.message);
 end
 
 % 7. Final lift
 fprintf('\n--- Final lift sequence ---\n');
 try
 final_lift = [robot_goal(1), robot_goal(2), lift_height, home(4:6)]; 
 
 qfinal = inverse_kinematics(final_lift(1), final_lift(2), final_lift(3), ...
 final_lift(4), final_lift(5), final_lift(6), q0);
 robot.movej(qfinal, 'joint'); % FIXED: Updated reference
 fprintf('✓ Final lift using IK successful\n');
 
 catch ME
 fprintf('✗ Final lift IK failed: %s\n', ME.message);
 try
 robot.movej(final_lift); % FIXED: Updated reference
 fprintf('✓ Final lift using Cartesian fallback successful\n');
 catch ME2
 fprintf('✗ Final lift Cartesian fallback also failed: %s\n', ME2.message);
 end
 end
 
 % 8. Return to home
 fprintf('\n--- Returning home ---\n');
 try
 robot.movej(home); % FIXED: Updated reference
 fprintf('✓ Successfully returned to home\n');
 catch ME
 fprintf('✗ Return to home failed: %s\n', ME.message);
 end
 
 fprintf('\n=== PART E COMPLETED ===\n');
 if failed_waypoints == 0
 fprintf('✓ ALL OPERATIONS SUCCESSFUL\n');
 else
 fprintf('⚠ COMPLETED WITH %d FAILED WAYPOINTS\n', failed_waypoints);
 end
end
function q_solution = inverse_kinematics(x, y, z, rx, ry, rz, q0)
 % Inverse kinematics function using RVC Toolbox
 
 % Convert inputs to proper units
 position = [x/1000, y/1000, z/1000]; % mm to meters
 orientation = [rx, ry, rz]; % Already in radians
 
 % Check for reasonable input ranges
 if norm(position) > 2.0 % 2 meter reach check
 warning('Target position may be outside robot workspace: %.3f m from origin', norm(position));
 end
 
 try
 % Build target homogeneous transform using RVC functions
 T_target = transl(position) * rpy2tr(orientation);
 
 catch ME
 error('Failed to build target transform: %s', ME.message);
 end
 % UR5e DH parameters using RVC Link class
 try
 % Using modified DH parameters for UR5e
 L(1) = Link([0, 0.1625, 0, pi/2], 'modified');
 L(2) = Link([0, 0, -0.425, 0], 'modified'); 
 L(3) = Link([0, 0, -0.3922, 0], 'modified');
 L(4) = Link([0, 0.1333, 0, pi/2], 'modified');
 L(5) = Link([0, 0.0997, 0, -pi/2], 'modified');
 L(6) = Link([0, 0.0996, 0, 0], 'modified');
 % Set realistic joint limits for UR5e
 for i = 1:6
 L(i).qlim = [-2*pi, 2*pi];
 end
 % Create robot model
 robot_model = SerialLink(L, 'name', 'UR5e');
 
 catch ME
 error('Failed to create robot model: %s', ME.message);
 end
 % Try inverse kinematics
 try
 q_solution = robot_model.ikine(T_target, q0, 'mask', [1 1 1 1 1 1]);
 
 % Check if solution is valid
 if any(isnan(q_solution))
 error('NaN values in solution');
 end
 
 % Verify the solution
 T_check = robot_model.fkine(q_solution);
 pos_error = norm(T_check.t - position');
 
 if pos_error > 0.01 % 1cm tolerance
 warning('Solution accuracy may be insufficient: %.6f m position error', pos_error);
 end
 
 catch ME
 % Fallback: try with different initial guesses
 initial_guesses = [
 zeros(1,6);
 [-pi/2, -pi/2, 0, -pi/2, 0, 0];
 [0, -pi/2, pi/2, -pi/2, -pi/2, 0];
 ];
 
 solution_found = false;
 for j = 1:size(initial_guesses, 1)
 try
 q_temp = robot_model.ikine(T_target, initial_guesses(j,:), 'mask', [1 1 1 1 1 1]);
 
 if ~any(isnan(q_temp))
 T_check = robot_model.fkine(q_temp);
 pos_error = norm(T_check.t - position');
 
 if pos_error < 0.01
 q_solution = q_temp;
 solution_found = true;
 break;
 end
 end
 catch
 continue;
 end
 end
 
 if ~solution_found
 error('Could not find valid IK solution after trying multiple initial guesses');
 end
 end
 
 % Final joint limit check
 for i = 1:6
 if q_solution(i) < -2*pi || q_solution(i) > 2*pi
 q_solution(i) = wrapToPi(q_solution(i));
 end
 end
end
function wrapped = wrapToPi(angle)
 % Wrap angle to [-pi, pi] range
 wrapped = mod(angle + pi, 2*pi) - pi;
end
%% Part A Functions
function warpedImg = perspectiveTransform(I)
 % Detect ArUco markers
 [ids, locs] = readArucoMarker(I);
 
 % Perspective transformation
 marker_pts = zeros(4,2);
 
 marker_pts(3,:) = mean(locs(:,:,ids == 0), 1); % Marker 0
 marker_pts(4,:) = mean(locs(:,:,ids == 1), 1); % Marker 1
 marker_pts(1,:) = mean(locs(:,:,ids == 2), 1); % Marker 2
 marker_pts(2,:) = mean(locs(:,:,ids == 3), 1); % Marker 3
 
 worldPoints = [-990, 60; % Marker 0
 -230, 60; % Marker 1
 -990, -520; % Marker 2
 -230, -520]; % Marker 3
 
 tform = fitgeotrans(marker_pts, worldPoints, 'projective');
 
 % Calculate output size based on world coordinates
 x_range = [min(worldPoints(:,1)), max(worldPoints(:,1))];
 y_range = [min(worldPoints(:,2)), max(worldPoints(:,2))];
 width = abs(diff(x_range));
 height = abs(diff(y_range));
 
 % Create reference object for the output image
 R = imref2d([height width], x_range, y_range);
 warpedImg = imwarp(I, tform, 'OutputView', R);
end
function [warpedImg, start_pos, goal_pos, obstacles, tilt_angle] = detectObjects(warpedImg, item)
 % Detecting Objects
 [ids, locs, detectedFamily] = readArucoMarker(warpedImg);
 
 % Positions of markers to be used in other Parts
 start_pos = [];
 goal_pos = [];
 obstacles = [];
 
 tilt_angle_start = 0; % Initialize to default value
 tilt_angle_goal = 0; % Initialize to default value
 big_centers = [];
 
 numMarkers = length(ids);
 for i = 1:numMarkers
 loc = locs(:,:,i);
 
 % Display the marker ID and family
 disp("Detected marker ID, Family: " + ids(i) + ", " + detectedFamily(i))
 
 % Insert marker IDs
 center = mean(loc);
 if ids(i) == 4
 % Obstacle marker
 redDotPosition = [center, 10];
 obstacles = [obstacles; center];
 warpedImg = insertShape(warpedImg,"FilledCircle",redDotPosition,ShapeColor="red",Opacity=1);
 end
 
 if ids(i) == 5 && strcmp(item, 'small')
 % Small item start position
 blueDotPosition = [center, 10];
 start_pos = center;
 corners = squeeze(locs(:,:,i));
 edges = diff(corners([1:4,1],:)); % 4 edges
 tilt_angle_start = atan2d(edges(1,2), edges(1,1));
 warpedImg = insertShape(warpedImg,"FilledCircle",blueDotPosition,ShapeColor="blue",Opacity=1);
 end
 
 if ids(i) == 7 && strcmp(item, 'small')
 % Small item goal position
 offset_distance = -80; % mm as per spec
 
 % This is for calculating the offset angle
 corners = squeeze(locs(:,:,i));
 edges = diff(corners([1:4,1],:)); % 4 edges (5th point repeats first)
 
 % Calculate overall tilt angle (relative to horizontal)
 tilt_angle_goal = atan2d(edges(1,2), edges(1,1));
 disp(tilt_angle_goal);
 
 % Calculate new position
 new_x = center(1) + offset_distance * cosd(tilt_angle_goal);
 new_y = center(2) + offset_distance * sind(tilt_angle_goal);
 
 % Mark the new position (green dot)
 greenDotPosition = [new_x, new_y, 10]; % [x,y,radius]
 goal_pos = [new_x, new_y];
 warpedImg = insertShape(warpedImg, "FilledCircle", greenDotPosition, 'ShapeColor', 'green', 'Opacity', 1);
 end
 
 % Big item detection for Part D
 if ids(i) == 6 && strcmp(item, 'big')
 % Big item marker
 big_centers = [big_centers; center];
 blueDotPosition = [center, 10];
 corners = squeeze(locs(:,:,i));
 edges = diff(corners([1:4,1],:)); % 4 edges
 tilt_angle_start = atan2d(edges(1,2), edges(1,1));
 warpedImg = insertShape(warpedImg,"FilledCircle",blueDotPosition,ShapeColor="blue",Opacity=1);
 end
 if ids(i) == 7 && strcmp(item, 'big')
 % Big item goal position
 corners = squeeze(locs(:,:,i)); % Current marker's corners
 edges = diff(corners([1:4,1],:)); % Get edges
 tilt_angle_goal = atan2d(edges(1,2), edges(1,1)); % Marker's orientation angle
 offset_x = -45 * cosd(tilt_angle_goal) - 100 * sind(tilt_angle_goal);
 offset_y = -45 * sind(tilt_angle_goal) + 100 * cosd(tilt_angle_goal);
 
 % Calculate new position
 new_x = center(1) + offset_x;
 new_y = center(2) + offset_y;
 
 % Mark the final position (green dot)
 greenDotPosition = [new_x, new_y, 10];
 goal_pos = [new_x, new_y];
 warpedImg = insertShape(warpedImg, "FilledCircle", greenDotPosition, 'ShapeColor', 'green', 'Opacity', 1);
 end
 end
 
 % This is for if the big item is detected
 if size(big_centers, 1) == 2
 midpoint = mean(big_centers, 1);
 start_pos = midpoint; % If you want to use this as a start position
 end
 
 % Calculate relative tilt angle (goal relative to start)
 tilt_angle = tilt_angle_goal - tilt_angle_start;
 
 % Alternative calculation if you prefer angle from horizontal:
 % tilt_angle = 90 - tilt_angle_goal; % Uncomment if preferred
 
 disp(['Tilt angle: ', num2str(tilt_angle)]);
 disp(['Start angle: ', num2str(tilt_angle_start)]);
 disp(['Goal angle: ', num2str(tilt_angle_goal)]);
 % Final Image for Part A
 figure(3);
 imshow(warpedImg);
 title('Object Detection Results');
end
%% FOR PART B
function [Fx, Fy] = calculatePotentialField(X, Y, goal_pos, obstacles)
 % Parameters
 k_att = 1; % Attractive gain
 k_rep = 5000; % Repulsive gain
 d0 = 500; % Influence distance of obstacles
 % Calculate attractive force (points TOWARD goal)
 dx_att = (goal_pos(1) - X);
 dy_att = (goal_pos(2) - Y);
 dist_att = sqrt(dx_att.^2 + dy_att.^2);
 Fx_att = k_att * dx_att ./ (dist_att + eps);
 Fy_att = k_att * dy_att ./ (dist_att + eps);
 % Initialize repulsive forces
 Fx_rep = zeros(size(X));
 Fy_rep = zeros(size(X));
 % Calculate repulsive forces (points AWAY from obstacles)
 if ~isempty(obstacles)
 for k = 1:size(obstacles,1)
 dx_rep = X - obstacles(k,1);
 dy_rep = Y - obstacles(k,2);
 dist_rep = sqrt(dx_rep.^2 + dy_rep.^2);
 % Only consider obstacles within influence distance
 valid = (dist_rep <= d0) & (dist_rep > 0);
 scale = k_rep * (1./dist_rep(valid) - 1/d0) ./ (dist_rep(valid).^2);
 Fx_rep(valid) = Fx_rep(valid) + scale .* dx_rep(valid);
 Fy_rep(valid) = Fy_rep(valid) + scale .* dy_rep(valid);
 end
 end
 % Combine forces
 Fx = Fx_att + Fx_rep;
 Fy = Fy_att + Fy_rep;
 % Normalize
 F_mag = sqrt(Fx.^2 + Fy.^2);
 Fx = Fx ./ (F_mag + eps);
 Fy = Fy ./ (F_mag + eps);
end
% Path Planning
function path = planPath(start_pos, goal_pos, X, Y, Fx, Fy) 
 path = start_pos;
 current_pos = start_pos;
 step_size = 5; % mm (used for initial steps)
 max_steps = 500;
 tolerance = 10; % mm
 min_segment_length = 50; % mm minimum straight-line segment length
 
 % Initialize variables for straight-line segments
 segment_start = start_pos;
 current_direction = [0, 0];
 segment_length = 0;
 
 for step = 1:max_steps
 % Check if reached goal
 if norm(current_pos - goal_pos) < tolerance
 % Add final segment to goal
 if norm(current_pos - segment_start) > 0
 path = [path; goal_pos];
 end
 break;
 end
 
 % Find closest grid point
 [~, x_idx] = min(abs(X(1,:) - current_pos(1)));
 [~, y_idx] = min(abs(Y(:,1) - current_pos(2)));
 % Get force direction (already normalized)
 new_direction = [Fx(y_idx,x_idx), Fy(y_idx,x_idx)];
 new_direction = new_direction / (norm(new_direction) + eps);
 
 % If we don't have a current direction or it changed significantly
 if segment_length == 0 || ...
 acosd(dot(current_direction, new_direction)) > 15 % degrees
 % If we've accumulated enough distance in current direction
 if segment_length >= min_segment_length
 % Add the intermediate point to the path
 path = [path; current_pos];
 segment_start = current_pos;
 segment_length = 0;
 end
 current_direction = new_direction;
 end
 
 % Move in the current direction
 current_pos = current_pos + step_size * current_direction;
 segment_length = segment_length + step_size;
 end
 
 % Make sure we reach exactly the goal position
 if norm(path(end,:) - goal_pos) > tolerance
 path = [path; goal_pos];
 end
end
%% FOR PART C - Coordinate Transformation
function robot_pos = imageToRobot(image_pos)
 % Convert from image coordinates to robot base coordinates
 scale_x = 1.0; % mm/pixel in x
 scale_y = -1.0; % mm/pixel in y
 offset_x = -990; % mm - change based on marker 0 position world coordinate
 offset_y = 60; % mm
 
 robot_pos = zeros(size(image_pos));
 robot_pos(:,1) = image_pos(:,1) * scale_x + offset_x;
 robot_pos(:,2) = image_pos(:,2) * scale_y + offset_y;
end
