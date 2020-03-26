function [x_arr,y_arr,phi_arr] = walk()
    x_arr=[];
    y_arr=[];
    phi_arr=[];
    

    prompt = 'Enter linear velocity: ';
    velocity = input(prompt);
    v_robot = rospublisher('/mobile_base/commands/velocity') ;
    velmsg = rosmessage(v_robot);
    velmsg.Linear.X = velocity;
    velmsg.Angular.Z = 0;

    prompt = 'Enter time to walk this velocity: ';
    tics = input(prompt);
    if (tics == 0) || (velocity == 0)
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
        x_arr = [x_arr;x];
        y_arr = [y_arr;y];
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        phi = rad2deg(angles(1));
        disp("phi: ");
        disp(phi)
        phi_arr = [phi_arr;phi];
        subplot(3,1,1);
        scatter(x_arr,y_arr);
        title("Position")

        laser = rossubscriber('/scan');
        scan = receive(laser,3);
        subplot(3,1,2);
        plot(scan);
        title("laser scan")
        disp(scan);
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
        subplot(3,1,3); 
        imshow(readImage(img));
        title("Camera")
    end
    

    disp("Walking ended.")
    
end