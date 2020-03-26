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

function rotate360()
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
        velmsg.Angular.Z = 0.3;
        velmsg.Linear.X = 0;
        send(v_robot,velmsg);
        
        %get picture and find colors
        imsub = rossubscriber('/camera/rgb/image_raw');
        img = receive(imsub);
        I = readImage(img);
        imshow(I);
        grayI = rgb2gray(I);
        R = I(:,:,1);
        R = imsubtract(R,grayI);
        R_b = im2bw(R, 0.3);
        G = I(:,:,2);
        G = imsubtract(G,grayI);
        G_b = im2bw(G, 0.1);
        B = I(:,:,3);
        B = imsubtract(B,grayI);
        B_b = im2bw(B, 0.3);
        %checkColor(R_b,G_b,B_b);
        
        %get shapes
        N = {R_b,G_b,B_b};
        for n=1: length(N)
            [centers, radii, metric] = imfindcircles(N{n}, [10,300]);
            if isempty([centers, radii, metric])
            else
                color = checkColor(R_b,G_b,B_b);
                
                fprintf("%s circle with %f radii.\n", color, max(radii));
                viscircles(centers,radii,"EdgeColor","black");
            end
  
        end
        
        BW1 = edge(grayI,'Canny', 0.04);
        %BW2 = edge(I,'Prewitt');
        imshowpair(I,BW1, "blend")
        
    end
    velmsg.Angular.Z = 0.0;
end

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