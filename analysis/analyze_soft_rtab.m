%% analyze_rtab.m
% Analysis for RTAB-Map results

close all, clear all

%% Load SOFT Odometry results
load("soft_pose.mat");

%% Analysis on data with imu
bag = rosbag('kitti_data_imu.bag');

gps = select(bag,'Topic','/kitti/oxts/gps/fix');
rtab_odom = select(bag,'Topic','/rtabmap/odom');
viso_odom = select(bag,'Topic','/viso2_ros/odometry');
rtab_pose = select(bag,'Topic','/rtabmap/localization_pose');

gps_msgs = readMessages(gps,'DataFormat','struct'); 
rtab_odom_msgs = readMessages(rtab_odom,'DataFormat','struct');
rtab_pose_msgs = readMessages(rtab_pose,'DataFormat','struct');

figObj =  findobj('type','figure');
figIdx = length(figObj) + 1;

plot_odom_pose(rtab_odom_msgs, figIdx);
plot_pose(rtab_pose_msgs, figIdx);
plot_soft(pose, figIdx);



% figObj =  findobj('type','figure');
% figIdx = length(figObj) + 1;
plot_gps(gps_msgs, figIdx);
legend("RTAB Odom", "RTAB-Map Pose", "SOFT pose", "GPS Fix")
title("Pose Localization Comparison Between vSLAM Packages")

%% Functions

function plot_odom_pose(msgs, figIdx)
    figure(figIdx)
    xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgs);
    yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgs);
    plot(xPoints,yPoints), hold on
    xlabel("X (m)"), ylabel("Y (m)")
end

function plot_pose(msgs, figIdx)
    figure(figIdx)
    xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgs);
    yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgs);
    
    % Create rotation matrix
    theta = 24; % to rotate counterclockwise and align with vSLAM
    R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
    % Rotate your point(s)
    point = [xPoints yPoints]'; % arbitrarily selected
    rotpoint = R*point;
    xPoints = rotpoint(1,:);
    yPoints = rotpoint(2,:);
    
    plot(xPoints,yPoints), hold on
    xlabel("X (m)"), ylabel("Y (m)")
end

function plot_gps(msgs, figIdx)
    figure(figIdx)
    lat = cellfun(@(m) double(m.Latitude),msgs);
    lon = cellfun(@(m) double(m.Longitude),msgs);
    alt = cellfun(@(m) double(m.Altitude),msgs);
    
    lla = [lat lon alt];
    llo = [lat(1) lon(1)];
    
    pts = lla2flat(lla, llo, 5, alt(1));
    xPoints = pts(:,1);
    yPoints = abs(pts(:,2));
    
    plot(xPoints,yPoints)
    xlabel("X(m)"), ylabel("Y(m)")
end

function plot_soft(pose, figIdx)
    figure(figIdx)
    xPoints = pose(:,3);
    yPoints = pose(:,1);
    
    plot(xPoints,yPoints), hold on
    xlabel("X (m)"), ylabel("Y (m)")

end
