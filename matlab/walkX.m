function  walk(x)    

    velocity = 0.1;
    v_robot = rospublisher('/mobile_base/commands/velocity') ;
    velmsg = rosmessage(v_robot);
    velmsg.Linear.X = velocity;
    velmsg.Angular.Z = 0;

    tics = x;
    if (tics == 0)
        return;
    end
    tic;
    while toc < tics
        send(v_robot,velmsg);

        odom = rossubscriber('/odom');
        odomdata = receive(odom,3);
        pose = odomdata.Pose.Pose;
        x = pose.Position.X;
        y = pose.Position.Y;
        z = pose.Position.Z;

        data = readCartesian(scan);
        x = data(:,1);
        y = data(:,2);
        dist = sqrt(x.^2 + y.^2);
        minDist = min(dist); 
        disp(minDist)
        if minDist < 0.5
            prompt = 'I am stuck, enter spin velocity: ';
            spinVelocity = input(prompt);
            prompt = "Enter time to spin: ";
            spinTime = input(prompt);
            tic;
            while toc < spinTime
                velmsg.Angular.Z = spinVelocity;
                velmsg.Linear.X = 0;
                send(v_robot,velmsg);
            end
            velmsg.Angular.Z = 0.0;
            velmsg.Linear.X = velocity;

        end

        imsub = rossubscriber('/camera/rgb/image_raw');
        img = receive(imsub);
        imshow(readImage(img));
    end
    

    disp("Walking ended.")
    
end