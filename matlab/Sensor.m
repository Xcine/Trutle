classdef Sensor
    %SENSOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        odom = rossubscriber('/odom');
        laser = rossubscriber('/scan');
        imsub = rossubscriber('/camera/rgb/image_raw');
    end
    
    methods(Static)
        function obj = Sensor()
            %SENSOR Construct an instance of this class
            %   Detailed explanation goes here
            
            figure;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
        
        function [x,y] = get_robot_position()
            odomdata = receive(Sensor.odom,3);
            pose = odomdata.Pose.Pose;
            x = pose.Position.X;
            y = pose.Position.Y;
        end
        
        function phi = get_robot_phi()
            odomdata = receive(Sensor.odom,3);
            pose = odomdata.Pose.Pose;
            quat = pose.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            phi = rad2deg(angles(1));
        end
        
        function min_dist = get_min_dist_of_laser()
            scan = receive(Sensor.laser,3);
            data = readCartesian(scan);
            x = data(:,1);
            y = data(:,2); 
            dist = sqrt(x.^2 + y.^2);
            min_dist = min(dist);

            %[r_x,r_y] = get_position()
        end
        
        function color = checkColor(r,g,b)
            if ismember(1,r)
                %disp("I see red");
                color = "red";
            elseif ismember(1,g)
                %disp("I see green");
                color = "green";
            elseif ismember(1,b)
                %disp("I see blue");
                color = "blue";
            else
                color = "NaN";
            end
        end
        
        function phi = transformPhi(phi, goOver)
            if phi > 360.0
                while phi > 360.0
                    phi = phi - 360.0;
                end
            else
                if goOver
                    while phi < 40.0
                        phi = phi + 360.0;
                    end
                else
                    while phi < 0.0
                        phi = phi + 360.0;
                    end
                end
            end
        end
        
        function I = get_camera_img()
            img = receive(Sensor.imsub);
            I = readImage(img);
        end
        
        function [R_b,G_b,B_b] = get_rgb_of_img(I)
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
        end
        
        function show_camera()
            imshow(Sensor.get_camera_img());
        end
        
        function [max_pos,max_rad] = get_max_pos_and_rad_in_img()
            I = Sensor.get_camera_img();
            imshow(I);
            [R_b,G_b,B_b] = Sensor.get_rgb_of_img(I);
            
            N = {R_b,B_b};
            max_pos = -1;
            max_rad = -1;
            for n=1: length(N)
                [centers, radii, metric] = imfindcircles(N{n}, [10,300]);
                if isempty([centers, radii, metric])
                else
                    cur_radii = max(radii);
                    id_max = find(radii == max(radii(:)));
                    cur_position = centers(id_max);
                    if cur_radii > max_rad
                        max_rad = cur_radii;
                        max_pos = cur_position;
                    end
                    %viscircles(centers,radii,"EdgeColor","black");
                end

            end
        end
        
        function position = get_position_of_near_candy_in_img()
            %returns -1 if no candy in image. Image is 640-480. position is
            %returned is the width pixel in image.

            [position,rad] = Sensor.get_max_pos_and_rad_in_img();
        end
        
    end
end

