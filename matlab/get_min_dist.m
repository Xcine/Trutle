function minDist = get_min_dist()
    laser = rossubscriber('/scan');
    scan = receive(laser,3);
    %Eating the candy and obstacle avoidance
    data = readCartesian(scan);
    x = data(:,1);
    y = data(:,2);
    % Compute distance of the closest candy 
    dist = sqrt(x.^2 + y.^2);
    minDist = min(dist);
    
    [r_x,r_y] = get_position()
    
    
    
end