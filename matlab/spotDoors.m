function spotDoors()
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
    while phi_dif < 360.0
        %read actual phi
        odom = rossubscriber('/odom');
        odomdata = receive(odom,3);
        pose = odomdata.Pose.Pose;
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        phi = rad2deg(angles(1));
        %fprintf('phi: %f.\n',phi);
        if toc < 5
            phi_dif = transformPhi(phi - phi_first, false);
        else
            phi_dif = transformPhi(phi - phi_first, true);
        end
        %fprintf('phi dif: %f.\n',phi_dif);
        
        %give spin vel
        v_robot = rospublisher('/mobile_base/commands/velocity') ;
        velmsg = rosmessage(v_robot);
        velmsg.Angular.Z = 0.3;
        velmsg.Linear.X = 0;
        send(v_robot,velmsg);
        
        %get picture and find lines
        imsub = rossubscriber('/camera/rgb/image_raw');
        img = receive(imsub);
        I = readImage(img);
        imshow(I);
        grayI = rgb2gray(I);        
        BW1 = edge(grayI,'Canny', 0.04);
        
        [H, T, R] = hough(BW1);
        lines = houghlines(BW1, R, T, H);
        for k = 1:length(lines)
            xy=[lines(k).point1; lines(k).point2];
            plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green')

            plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow')
            plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red')

            len = norm(lines(k).point1-lines(k).point2);
            if (len > max_len)
                max_len = len;
                xy_long = xy;
            end
        end
        
        %door recognition
              
    end
    velmsg.Angular.Z = 0.0;
end