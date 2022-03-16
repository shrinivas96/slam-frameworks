function [pntsMap] = world_to_map_coordinates(pntsWorld, gridSize, offset)
% Convert points from the world coordinates frame to the map frame.
% pntsWorld is a matrix of N points with each column representing a point in world coordinates (meters).
% gridSize is the size of each grid in meters.
% offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
% when converting to map coordinates.
% pntsMap is a 2xN matrix containing the corresponding points in map coordinates.

% TODO: compute pntsMap

% repeat the offset matrix to reflect the size of pntsWorld. Each column in
% pntsWorld is a point in the robot's frame, and offset is the global
% origin in mapframe (before converting to grids)
repeatOffset = repmat(offset, 1, size(pntsWorld, 2));


% this is essentially the distance between each coordinate of a point the
% robot frame and their respective axes, defined by the offset values.
% Subtracting by the offset from this point tells us how far is this point
% from the map frame XY axes.
dist2axes = pntsWorld - repeatOffset;     


% fidn the corresponding box where this point needs to be placed in the map
pntsMap = floor(dist2axes ./ gridSize);

end
