clc; clear all;

% Load laser scans and robot poses.
load("../data/laser")
laser = read_robotlaser('../data/csail.log');

% Extract robot poses: Nx3 matrix where each row is in the form: [x y theta]
poses = [laser.pose];
poses = reshape(poses,3,size(poses,2)/3)';

gridSize = 0.5;

border = 30;
robXMin = min(poses(:,1));
robXMax = max(poses(:,1));
robYMin = min(poses(:,2));
robYMax = max(poses(:,2));
mapBox = [robXMin-border robXMax+border robYMin-border robYMax+border];

mapSizeMeters = [mapBox(2)-mapBox(1) mapBox(4)-mapBox(3)];
mapSize = ceil([mapSizeMeters/gridSize]);


x = [poses(:, 1); mapBox(1:2)'];
y = [poses(:, 2); mapBox(3:4)'];

scatter(x, y);