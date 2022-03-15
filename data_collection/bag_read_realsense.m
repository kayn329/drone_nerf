clc; clear; close all;

%% Load rosbag
filename = "2022-02-17-15-03-48.bag";
bag = rosbag(filename);

%% Camera pose
camera_pose_sel = select(bag,'Topic','/realsense/mavros/vision_pose/pose');
camera_pose_msg = readMessages(camera_pose_sel,'DataFormat','struct');

n = size(camera_pose_msg,1);
camera_poses = zeros(8,n);
for k = 1:n
    % time
    camera_poses(1,k) = double(camera_pose_msg{k}.Header.Stamp.Sec) + double(camera_pose_msg{k}.Header.Stamp.Nsec)*10^(-9);
    % POSITION
    camera_poses(2,k) = camera_pose_msg{k}.Pose.Position.X;
    camera_poses(3,k) = camera_pose_msg{k}.Pose.Position.Y;
    camera_poses(4,k) = camera_pose_msg{k}.Pose.Position.Z;
    % ORIENTATION (QUATERNIONS)
    camera_poses(5,k) = camera_pose_msg{k}.Pose.Orientation.X;
    camera_poses(6,k) = camera_pose_msg{k}.Pose.Orientation.Y;
    camera_poses(7,k) = camera_pose_msg{k}.Pose.Orientation.Z;
    camera_poses(8,k) = camera_pose_msg{k}.Pose.Orientation.W;
end

%% Target pose
target_pose_sel = select(bag,'Topic','/preston/mavros/vision_pose/pose');
target_pose_msg = readMessages(target_pose_sel,'DataFormat','struct');

n = size(target_pose_msg,1);
target_poses = zeros(8,n);
for k = 1:n
    % time
    target_poses(1,k) = double(target_pose_msg{k}.Header.Stamp.Sec) + double(target_pose_msg{k}.Header.Stamp.Nsec)*10^(-9);
    % POSITION
    target_poses(2,k) = target_pose_msg{k}.Pose.Position.X;
    target_poses(3,k) = target_pose_msg{k}.Pose.Position.Y;
    target_poses(4,k) = target_pose_msg{k}.Pose.Position.Z;
    % ORIENTATION (QUATERNIONS)
    target_poses(5,k) = target_pose_msg{k}.Pose.Orientation.X;
    target_poses(6,k) = target_pose_msg{k}.Pose.Orientation.Y;
    target_poses(7,k) = target_pose_msg{k}.Pose.Orientation.Z;
    target_poses(8,k) = target_pose_msg{k}.Pose.Orientation.W;
end

%% Camera image
image_sel = select(bag,'Topic','/camera/color/image_raw');
image_msg = readMessages(image_sel);
height = 480;
width = 640;
n = size(image_msg,1);
image_time_vec = zeros(1,n);
for k = 1:n
    [image_rgb, ~] = readImage(image_msg{k});
    image_time_vec(k) = double(image_msg{k}.Header.Stamp.Sec) + double(image_msg{k}.Header.Stamp.Nsec)*10^(-9);
%      imwrite(image_rgb, strcat('./robot_drone_2/image', sprintf('%04d', k), '.png'))
end
%imshow(image_rgb)

%% Time Sync
target_poses_sync = zeros(length(image_time_vec), 7);
camera_poses_sync = zeros(length(image_time_vec), 7);
for k = 1:length(image_time_vec)
    t = image_time_vec(k);
    [target_val, target_idx] = min(abs(target_poses(1,:) - t));
    [camera_val, camera_idx] = min(abs(camera_poses(1,:) - t));
    target_poses_sync(k,:) = target_poses(2:8, target_idx);
    camera_poses_sync(k,:) = camera_poses(2:8, camera_idx);
%     save(strcat('./robot_drone_2/target_pose', sprintf('%04d', k),'.txt'), 'target_poses_sync', '-ascii', '-double', '-tabs')
%     save(strcat('./robot_drone_2/drone_pose', sprintf('%04d', k),'.txt'), 'camera_poses_sync', '-ascii', '-double', '-tabs')
end

%writematrix(target_poses, './robot_drone_1/target_poses.csv')
%writematrix(drone_poses, './robot_drone_1/drone_poses.csv')

%% PLOTTING
figure % POSITION
subplot(3,1,1)
plot(target_poses(1,:), target_poses(2,:), '--')
hold on
plot(camera_poses(1,:), camera_poses(2,:), '--')
ylabel('Position - x')
legend('Target', 'Camera')
title('POSITION')
subplot(3,1,2)
plot(target_poses(1,:), target_poses(3,:), '--')
hold on
plot(camera_poses(1,:), camera_poses(3,:), '--')
ylabel('Position - y')
subplot(3,1,3)
plot(target_poses(1,:), target_poses(4,:), '--')
hold on
plot(camera_poses(1,:), camera_poses(4,:), '--')
ylabel('Position - z')
xlabel('Epoch Time (sec)')

figure % TRAJECTORY
plot(camera_poses(2,:), camera_poses(3,:))
