function max_radii = select_candy_from_all2()
    max_phi = -1;
    max_radii = -1;
    
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
    while toc < 20
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
        fprintf('phi dif: %f.\n',phi_dif);
        
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
        N = {R_b,B_b};
        for n=1: length(N)
            [centers, radii, metric] = imfindcircles(N{n}, [10,300]);
            if isempty([centers, radii, metric])
            else
                id_max = find(radii == max(radii(:)));
                cur_position = centers(id_max);
                cur_radii = max(radii);
                if cur_radii > max_radii
                    max_radii = cur_radii;
                    position = cur_position;
                    max_phi = phi;
                end
                %viscircles(centers,radii,"EdgeColor","black");
            end
  
        end
        
        %give spin vel
        v_robot = rospublisher('/mobile_base/commands/velocity') ;
        velmsg = rosmessage(v_robot);
        velmsg.Angular.Z = 0.8;
        velmsg.Linear.X = 0;
        send(v_robot,velmsg);

        
    end
    velmsg.Angular.Z = 0.0;
end