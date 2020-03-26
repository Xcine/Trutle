function rotateX(x,dir)
    %read first phi
    odom = rossubscriber('/odom');
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    phi_first = rad2deg(angles(1))-1;
    %fprintf('first phi: %f.\n',phi_first);
    phi_dif = 0.0;
    tic;
    while phi_dif < x
        %read actual phi
        odom = rossubscriber('/odom');
        odomdata = receive(odom,3);
        pose = odomdata.Pose.Pose;
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        phi = rad2deg(angles(1));
        fprintf('phi: %f.\n',phi);
        if toc < 5
            phi_dif = transformPhi(phi - phi_first, false);
        else
            phi_dif = transformPhi(phi - phi_first, true);
        end
        fprintf('phi dif: %f.\n',phi_dif);
        
        %give spin vel
        v_robot = rospublisher('/mobile_base/commands/velocity') ;
        velmsg = rosmessage(v_robot);
        velmsg.Angular.Z = dir*0.3;
        velmsg.Linear.X = 0;
        send(v_robot,velmsg);  
        
        imsub = rossubscriber('/camera/rgb/image_raw');
        img = receive(imsub);
        I = readImage(img);
        imshow(I);
    end
    velmsg.Angular.Z = 0.0;
end