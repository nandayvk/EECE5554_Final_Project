close all, clear all

bag = rosbag('kitti_data.bag');

gps = select(bag,'Topic','/kitti/oxts/gps/fix');
rtab_odom = select(bag,'Topic','/rtabmap/odom');
viso_odom = select(bag,'Topic','/viso2_ros/odometry');

gps_msgs = readMessages(gps,'DataFormat','struct'); 
rtab_msgs = readMessages(rtab_odom,'DataFormat','struct');
viso_msgs = readMessages(viso_odom,'DataFormat','struct');

figObj =  findobj('type','figure');
figIdx = length(figObj) + 1;

plot_odom_pose(rtab_msgs, "RTAB", figIdx);
plot_odom_pose(viso_msgs, "Viso", figIdx);
legend("RTAB", "Viso")
title("Pose Localization from Visual Odometry")

figObj =  findobj('type','figure');
figIdx = length(figObj) + 1;
plot_gps(gps_msgs, figIdx);

function plot_odom_pose(msgs, name, figIdx)
    figure(figIdx)
    xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgs);
    yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgs);
    plot(xPoints,yPoints), hold on
    xlabel("X (m)"), ylabel("Y (m)")
end

function plot_gps(msgs, figIdx)
    figure(figIdx)
    xPoints = cellfun(@(m) double(m.Latitude),msgs);
    yPoints = cellfun(@(m) double(m.Longitude),msgs);
    plot(xPoints,yPoints)
    title("GPS Fix")
    xlabel("lat"), ylabel("lon")
end