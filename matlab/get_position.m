function [x,y] = get_position()
    odom = rossubscriber('/odom');
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
end