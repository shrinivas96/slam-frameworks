function plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t)

    close all
    % figure 
    set(0, 'DefaultFigureVisible', 'off')
    figure(1);
    axis(mapBox);
    hold on;
	map = map';
	imshow(ones(size(map)) - log_odds_to_prob(map))
    mapSize = size(map);
    s = mapSize(1:2); 

    set(gcf, "position", [50 50 s*5]) 
    set(gca, "position", [.05 .05 .9 .9]) 
	hold on;
    traj = [poses(1:t,1)';poses(1:t,2)'];
    traj = world_to_map_coordinates(traj, gridSize, offset);
    plot(traj(1,:),traj(2,:), 'g')
    plot(robPoseMapFrame(1),robPoseMapFrame(2),'bo','markersize',5,'linewidth',4)
    plot(laserEndPntsMapFrame(1,:),laserEndPntsMapFrame(2,:),'ro','markersize',2)
    filename = sprintf('plots/gridmap_%03d.png', t);
    print(filename, '-dpng');
    close all;
end


