function [mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(map, scan, robPose, gridSize, offset, probOcc, probFree)
% Compute the log odds values that should be added to the map based on the inverse sensor model
% of a laser range finder.

% map is the matrix containing the occupancy values (IN LOG ODDS) of each cell in the map.
% scan is a laser scan made at this time step. Contains the range readings of each laser beam.
% robPose is the robot pose in the world coordinates frame.
% gridSize is the size of each grid in meters.
% offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
% when converting to map coordinates.
% probOcc is the probability that a cell is occupied by an obstacle given that a
% laser beam endpoint hit that cell.
% probFree is the probability that a cell is occupied given that a laser beam passed through it.

% mapUpdate is a matrix of the same size as map. It has the log odds values that need to be added for the cells
% affected by the current laser scan. All unaffected cells should be zeros.
% robPoseMapFrame is the pose of the robot in the map coordinates frame.
% laserEndPntsMapFrame are map coordinates of the endpoints of each laser beam (also used for visualization purposes).

% basic config
laseMaxRange = 30;
subsample = false;

% Initialize mapUpdate.
mapUpdate = zeros(size(map));

% Robot pose as a homogeneous transformation matrix.
robTrans = v2t(robPose);


% TODO: compute robPoseMapFrame. Use your world_to_map_coordinates implementation.
robPoseMapFrame = world_to_map_coordinates(robPose(1:2, :), gridSize, offset);
robPoseMapFrame(3) = robPose(3);

% Compute the Cartesian coordinates of the laser beam endpoints.
% Set the third argument to 'true' to use only half the beams for speeding up the algorithm when debugging.
laserEndPnts = robotlaser_as_cartesian(scan, laseMaxRange, subsample);

% Compute the endpoints of the laser beams in the world coordinates frame.
laserEndPnts = robTrans*laserEndPnts;

% TODO: compute laserEndPntsMapFrame from laserEndPnts. Use your world_to_map_coordinates implementation.
laserEndPntsMapFrame = world_to_map_coordinates(laserEndPnts(1:2, : ), gridSize, offset);

% freeCells are the map coordinates of the cells through which the laser beams pass.
freeCells = [];

% Iterate over each laser beam and compute freeCells.
% Use the bresenham method available to you in tools for computing the X and Y
% coordinates of the points that lie on a line.
% Example use for a line between points p1 and p2:
% [X,Y] = bresenham(map,[p1_x, p1_y; p2_x, p2_y]);
% You only need the X and Y outputs of this function.
for sc=1:size(laserEndPntsMapFrame, 2)
    
    %TODO: compute the XY map coordinates of the free cells along the laser beam ending in laserEndPntsMapFrame(:,sc)
    freeLine = [robPoseMapFrame(1:2, 1)'; laserEndPntsMapFrame(:, sc)'];
    
    [X, Y] = bresenham(freeLine);
    % [Xte, Yte] = bresenham_line(freeLine);
    
    %TODO: add them to freeCells
    freeCells = [freeCells, [X; Y]];
end

%TODO: update the log odds values in mapUpdate for each free cell according to probFree.

for fc = 1:size(freeCells, 2)
    fcX = freeCells(1, fc);
    fcY = freeCells(2, fc);
    mapUpdate(fcX, fcY) = prob_to_log_odds(probFree);
end

%TODO: update the log odds values in mapUpdate for each laser endpoint according to probOcc.

for sc=1:size(laserEndPntsMapFrame, 2)
    scX = laserEndPntsMapFrame(1, sc);
    scY = laserEndPntsMapFrame(2, sc);
    
    mapUpdate(scX, scY) = prob_to_log_odds(probOcc);
end
end
