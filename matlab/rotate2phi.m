function rotate2phi(phi)
    %read first phi
    odom = rossubscriber('/odom');
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    phi_first = rad2deg(angles(1))-1;
    tic;
    while abs(phi_first - phi) > 80
        %read actual phi
        odom = rossubscriber('/odom');
        odomdata = receive(odom,3);
        pose = odomdata.Pose.Pose;
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        phi = rad2deg(angles(1));
        fprintf('phi: %f.\n',phi);
        
        if phi_first - phi < 0 
            dir = -1.0;
        else
            dir = 1.0;
        end
      
        %give spin vel
        v_robot = rospublisher('/mobile_base/commands/velocity') ;
        velmsg = rosmessage(v_robot);
        velmsg.Angular.Z = 0.8*dir;
        velmsg.Linear.X = 0;
        send(v_robot,velmsg);
        
        %get picture and find colors
        imsub = rossubscriber('/camera/rgb/image_raw');
        img = receive(imsub);
        I = readImage(img);
        imshow(I);

        
    end
    velmsg.Angular.Z = 0.0;
end