classdef Sensor
    %SENSOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        odom = rossubscriber('/odom');
        laser = rossubscriber('/scan');
        imsub = rossubscriber('/camera/rgb/image_raw');
        gazebo = ExampleHelperGazeboCommunicator;
        robot = ExampleHelperGazeboSpawnedModel("mobile_base",Sensor.gazebo);
    end
    
    methods(Static)
        function obj = Sensor()
            %SENSOR Construct an instance of this class
            %   Detailed explanation goes here
            
            figure;
        end
        
        function [x,y] = get_relative_robot_position()
            odomdata = receive(Sensor.odom,3);
            pose = odomdata.Pose.Pose;
            x = pose.Position.X;
            y = pose.Position.Y;
        end
        
        function [x,y] = get_robot_position()
            position = Sensor.robot.getState();
            x = position(1);
            y = position(2);
        end
        
        function dist = get_dist_of_2_positions(x1,y1,x2,y2)
            dist = sqrt((x1-x2)^2 + (y1-y2)^2);
        end
        
        function [x,y] = get_position_of_gazebo_obj(name)
            object = ExampleHelperGazeboSpawnedModel(name,Sensor.gazebo);
            position = object.getState();
            x = position(1);
            y = position(2);
        end
        
        function balls = get_gazebo_balls()
            cur_state = getSpawnedModels(Sensor.gazebo);
            balls = [];
            for n = 1: length(cur_state)
                if contains(cur_state{n},"Ball")
                    balls = [balls;convertCharsToStrings(cur_state{n})];
                end
            end
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
        
        function show_laser_scan()
            scan = receive(Sensor.laser,3);
            subplot(2,1,2);
            plot(scan);
        end
        
        function [beta,y,x] = get_beta_y_to_parallel_door(alpha,c,b)
            a = sqrt(b^2 + c^2 -2*b*c*cos(alpha));
            beta = asin((sin(alpha)*b)/a);
            y = cos(beta)*c;
            x = sin(beta)*c;
        end
        
        function ranges = smooth_nans(ranges)
            smooth1=[0 1 0];
            smooth2=[0 1 1 0];
            smooth3=[0 1 1 1 0];
            nan_ranges = reshape(isnan(ranges), [1,640]);
            k1 = strfind(nan_ranges,smooth1);
            k2 = strfind(nan_ranges,smooth2);   
            k3 = strfind(nan_ranges,smooth3);
            
            for n=1: length(k1)
                new_value = ranges(k1(n));
                ranges(k1(n)+1) = new_value;
                ranges(k1(n)+1) = new_value;
            end
            
            for n=1: length(k2)
                new_value = ranges(k2(n));
                ranges(k2(n)+1) = new_value;
                ranges(k2(n)+2) = new_value;
            end
            
            for n=1: length(k3)
                new_value = ranges(k3(n));
                ranges(k3(n)+1) = new_value;
                ranges(k3(n)+2) = new_value;
                ranges(k3(n)+3) = new_value;
            end
        end
        
        function [start_nan, end_nan, ranges] = after_door_nans()
            scan = receive(Sensor.laser,3);
            subplot(2,1,2);
            plot(scan);
            Sensor.show_camera();
            ranges = scan.Ranges;
            
            ranges = Sensor.smooth_nans(ranges);
            
            jump_id_right = -1;
            jump_id_left = -1;
            len_ranges = length(ranges);
            for n=1:len_ranges-2
                dif1 = abs(ranges(n)-ranges(n+1));
                dif2 = abs(ranges(len_ranges - n)-ranges(len_ranges - n-1));
                if dif1 > 0.6 && jump_id_right == -1
                    jump_id_right = n;
                end
                if dif2 > 0.6 && jump_id_right == -1
                    jump_id_left = len_ranges-n;
                end
            end
            
            nan_ranges = reshape(isnan(ranges), [1,640]);
            
            nans = regionprops(nan_ranges, 'Area');
            allAreas = max([nans.Area]);
            if isempty(allAreas)
                start_nan = 1;
                end_nan = 640;
            else
                onex = ones([1 allAreas]);
                start_nan = strfind(nan_ranges,onex);
                %start_nan = sum(bwareafilt(nan_ranges==0, 1));
                end_nan = allAreas + start_nan;
            end
            
            if (jump_id_right ~= -1) && (jump_id_left ~= -1) && abs(jump_id_right-jump_id_left)<4
                if start_nan < jump_id_right
                    end_nan = jump_id_left+1;
                    allAreas = (end_nan-start_nan)/2.0;
                else
                    start_nan = jump_id_right;
                    allAreas = (end_nan-start_nan)/2.0;
                end
            elseif (jump_id_right ~= -1) && (jump_id_left ~= -1)
                start_nan = jump_id_right;
                end_nan = jump_id_left+1;
                allAreas = (end_nan-start_nan)/2.0;
            elseif (jump_id_left ~= -1)
                end_nan = jump_id_left+1;
                allAreas = (end_nan-start_nan)/2.0;
            elseif (jump_id_right) ~= -1
                start_nan = jump_id_right;
                allAreas = (end_nan-start_nan)/2.0;
            end
            
            fprintf("Door(s,e,A): (%f, %f, %f)\n", start_nan, allAreas, end_nan);
            
        end
        
        function position = door_in_laser()
            scan = receive(Sensor.laser,3);
            subplot(2,1,2);
            plot(scan);
            Sensor.show_camera();
            ranges = scan.Ranges;
            position = -1;
  
            ranges = Sensor.smooth_nans(ranges);
            
            jump_id_right = -1;
            jump_id_left = -1;
            len_ranges = length(ranges);
            for n=1:len_ranges-2
                dif1 = abs(ranges(n)-ranges(n+1));
                dif2 = abs(ranges(len_ranges - n)-ranges(len_ranges - n-1));
                if dif1 > 0.6 && jump_id_right == -1
                    jump_id_right = n;
                end
                if dif2 > 0.6 && jump_id_right == -1
                    jump_id_left = len_ranges-n;
                end
            end
            
            nan_ranges = reshape(isnan(ranges), [1,640]);
            
            nans = regionprops(nan_ranges, 'Area');
            allAreas = max([nans.Area]);
            if isempty(allAreas)
                start_nan = 1;
                end_nan = 640;
            else
                onex = ones([1 allAreas]);
                start_nan = strfind(nan_ranges,onex);
                %start_nan = sum(bwareafilt(nan_ranges==0, 1));
                end_nan = allAreas + start_nan;
            end
            
            if (jump_id_right ~= -1) && (jump_id_left ~= -1) && abs(jump_id_right-jump_id_left)<4
                if start_nan < jump_id_right
                    end_nan = jump_id_left+1;
                    allAreas = (end_nan-start_nan)/2.0;
                else
                    start_nan = jump_id_right;
                    allAreas = (end_nan-start_nan)/2.0;
                end
            elseif (jump_id_right ~= -1) && (jump_id_left ~= -1)
                start_nan = jump_id_right;
                end_nan = jump_id_left+1;
                allAreas = (end_nan-start_nan)/2.0;
            elseif (jump_id_left ~= -1)
                end_nan = jump_id_left+1;
                allAreas = (end_nan-start_nan)/2.0;
            elseif (jump_id_right) ~= -1
                start_nan = jump_id_right;
                allAreas = (end_nan-start_nan)/2.0;
            end

            
            if (start_nan ~= 1) && (end_nan ~= 640)
                if (allAreas < 300)
                    fprintf("Door(s,e,A): (%f, %f, %f)\n", start_nan, allAreas, end_nan);
                    position = start_nan + allAreas/2.0;
                    fprintf("position: %f \n",position);
                    position = 640 - position;
                    line([position,position],[0,480]);
                end
            end
            
        end
        
        function bool = wall_in_sight()
            scan = receive(Sensor.laser,3);
            subplot(2,1,2);
            plot(scan);
            Sensor.show_camera();
            ranges = scan.Ranges;
            bool = false;
            
            if ranges(320)>0 
                bool = true;
            end
            
            
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
                    while phi < 20.0
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
            subplot(2,1,1);
            imshow(Sensor.get_camera_img());
        end
        
        function [max_pos,max_rad, max_id] = get_max_pos_and_rad_in_img()
            I = Sensor.get_camera_img();
            subplot(2,1,1)
            imshow(I);
            [R_b,G_b,B_b] = Sensor.get_rgb_of_img(I);
            
            N = {R_b,B_b};
            max_pos = -1;
            max_rad = -1;
            max_id = -1;
            for n=1: length(N)
                [centers, radii, metric] = imfindcircles(N{n}, [10,300]);
                if isempty([centers, radii, metric])
                else
                    [radii_sorted,sortIdx] = sort(radii,'descend');
                    centers_sorted = centers(sortIdx);
                    
                    cur_radii = max(radii);
                    id_max = find(radii == max(radii(:)));
                    cur_position = centers(id_max);
                    if cur_radii > max_rad
                        max_rad = cur_radii;
                        max_pos = cur_position;
                        max_id = id_max;
%                         radii
%                         centers
                    end
                    %viscircles(centers,radii,"EdgeColor","black");
                end

            end
        end
        
        function [pos_sorted,rad_sorted] = get_max_min_pos_and_rad_in_img()
            I = Sensor.get_camera_img();
            subplot(2,1,1)
            imshow(I);
            [R_b,G_b,B_b] = Sensor.get_rgb_of_img(I);
            
            N = {R_b,B_b};
            max_pos = -1;
            max_rad = -1;
            max_id = -1;
            
            empty1 = false;
            empty2 = false;
            
            [centers, radii, metric] = imfindcircles(N{1}, [10,300]);
            if isempty([centers, radii, metric])
                empty1 = true;
            else
                [radii_sorted,sortIdx] = sort(radii,'descend');
                centers_sorted = centers(sortIdx);
                %viscircles(centers,radii,"EdgeColor","black");
            end
            
            [centers, radii, metric] = imfindcircles(N{2}, [10,300]);
            if isempty([centers, radii, metric])
                empty2 = true;
            else
                [radii_sorted2,sortIdx] = sort(radii,'descend');
                centers_sorted2 = centers(sortIdx);
                %viscircles(centers,radii,"EdgeColor","black");
            end
            
            if empty1 == false && empty2 == false
                [rad_sorted,sortIdx] = sort([radii_sorted;radii_sorted2],'descend');
                pos = [centers_sorted;centers_sorted2];
                pos_sorted = pos(sortIdx);
            elseif empty1 == false && empty2 == true
                rad_sorted = radii_sorted;
                pos_sorted = centers_sorted;
            elseif empty1 == true && empty2 == false
                rad_sorted = radii_sorted2;
                pos_sorted = centers_sorted2;
            else
                rad_sorted = [];
                pos_sorted = [];
            end
        end
        
%         function [position,rad,max_id] = get_position_of_near_candy_in_img(last_id)
%             %returns -1 if no candy in image. Image is 640-480. position is
%             %returned is the width pixel in image.
%             
%             [position1,rad,max_id] = Sensor.get_max_pos_and_rad_in_img();
%             
%             if max_id >= last_id
%                 position = position1;
%             else
%                 position = Sensor.get_position_id_candy_in_img(last_id);
%             end
%         end

        function [position,rad] = get_position_of_near_candy_in_img()
            %returns -1 if no candy in image. Image is 640-480. position is
            %returned is the width pixel in image.
            
            [pos_sorted,rad_sorted] = Sensor.get_max_min_pos_and_rad_in_img();
            %rad_sorted
            %pos_sorted
            if isempty(pos_sorted)
                position = -1;
                rad = -1;
            else
                len_rad = length(rad_sorted);
                rad_max = rad_sorted(1);
                min_rad_in_area = rad_max;
                id=1;
                for n=1:len_rad
                    if abs(rad_sorted(n)-min_rad_in_area)<10 && rad_sorted(n)>12
                        min_rad_in_area = rad_sorted(n);
                        id = n;
                    end
                end
                
                if rad_sorted(id) > 12
                    position = pos_sorted(id);
                    rad = rad_sorted(id);
                    line([position,position],[0,480]);
                    fprintf("chosen: %f, %f\n",rad,position);
                else
                    position = -1;
                    rad = -1;
                end
            end
                
        end
        
%         function [position,rad] = get_position_of_near_candy_in_img()
%             %returns -1 if no candy in image. Image is 640-480. position is
%             %returned is the width pixel in image.
%             
%             [position,rad,max_id] = Sensor.get_max_pos_and_rad_in_img();
%         end
        
        function position = get_position_id_candy_in_img(id)
            I = Sensor.get_camera_img();
            subplot(2,1,1)
            imshow(I);
            [R_b,G_b,B_b] = Sensor.get_rgb_of_img(I);
            
            N = {R_b,B_b};
            position = -1;
            for n=1: length(N)
                [centers, radii, metric] = imfindcircles(N{n}, [10,300]);
                if isempty([centers, radii, metric])
                else
                    if id <= length(centers)
                        position = centers(id);
                    end
                    %viscircles(centers,radii,"EdgeColor","black");
                end

            end
        end
        
        function [min_dist,min_ball] = get_dist_and_name_of_nearest_candy()
            [r_x,r_y] = Sensor.get_robot_position();
            
            min_dist = 100;
            min_ball = " ";
            ball_names = Sensor.get_gazebo_balls();
            for n = 1: length(ball_names)
                [b_x,b_y] = Sensor.get_position_of_gazebo_obj(ball_names(n));
                cur_dist = sqrt((r_x-b_x)^2 + (r_y-b_y)^2);
                if cur_dist < min_dist
                    min_dist = cur_dist;
                    min_ball = ball_names(n);
                end
            end
            
        end
        
        function draw_rect1()
            I = Sensor.get_camera_img();
            grayI = rgb2gray(I);
            bw= grayI;
            bw = edge(grayI,'Sobel');
            bw = bw <150;
            %bw = bwareaopen(bw, 5000); 
            %bw = edge(grayI,'Canny', 0.04);
            stats = regionprops(bw);
            imshow(bw);
            
            id = -1;
            max_area = 0;
            c1 = 0;
            c2 = 0;
            numc = 0;
            for i = 1:numel(stats)
                cur_area = stats(i).Area;
                bbu = stats(i).BoundingBox(1);
                bb1 = stats(i).BoundingBox(3);
                bb2 = stats(i).BoundingBox(4);
                ratio = bb2/bb1;
                if (ratio > 1.1) && (bb1 > 20)
                    c1 = c1 + stats(i).Centroid(1);
                    c2 = c2 + stats(i).Centroid(2);
                    numc = numc + 1;
                end
                
                if (cur_area > max_area) && (ratio > 1.1) && (bb1 > 20)
                    max_area = cur_area;
                    id = i;
                end
            end
            c1
            c2
            c1 = c1/numc
            c2 = c2/numc
            hold on;
            plot(c1,c2, 'r+', 'MarkerSize', 30, 'LineWidth', 2);
           
            if id ~= -1
                rectangle('Position', stats(id).BoundingBox, ...
                'Linewidth', 3, 'EdgeColor', 'r', 'LineStyle', '--');
                stats(id)
            end
            
            for i = 1:numel(stats)
                rectangle('Position', stats(i).BoundingBox, ...
                'Linewidth', 1, 'EdgeColor', 'b', 'LineStyle', '--');
            end
            
            
        end
        
        function bool = cen_in_rect(bb,c1,c2)
            left_width = bb.BoundingBox(1);
            right_width = bb.BoundingBox(1) + bb.BoundingBox(3);
            up_hight = bb.BoundingBox(2);
            down_hight = bb.BoundingBox(2) + bb.BoundingBox(4);
            if c1 < right_width && c1 > left_width && c2 < down_hight && c2 > up_hight
                bool = true;
            else
                bool = false;
            end
        end
        
        function position = get_position_of_rect()
            I = Sensor.get_camera_img();
            grayI = rgb2gray(I);
            bw= imadjust(grayI);
            bw = edge(grayI,'Canny',0.04);
            %bw = bw <105;
            %bw = bwareaopen(bw, 50); 
            %bw = edge(grayI,'Canny', 0.04);
            stats = regionprops(bw, 'BoundingBox','Perimeter','Area','FilledArea','Solidity','Centroid');
            imshow(I);
            
            id = -1;
            max_area = 0;
            c1 = 0;
            c2 = 0;
            numc=0;
            position = -1;
%             for i = 1:numel(stats)
%                 cur_area = stats(i).Area;
%                 bbu = stats(i).BoundingBox(2);
%                 bb1 = stats(i).BoundingBox(3);
%                 bb2 = stats(i).BoundingBox(4);
%                 sol = stats(i).Solidity;
%                 ratio = bb2/bb1;
% %                 if (ratio > 1.1)% && (bb1 > 90)
% %                     c1 = c1 + stats(i).Centroid(1);
% %                     c2 = c2 + stats(i).Centroid(2);
% %                     numc = numc + 1;
% %                 end
%                 
%                 if (cur_area > max_area) && (ratio > 1.1) && (bb1 > 40) && (bbu+bb2>240)
%                     max_area = cur_area;
%                     id = i;
%                 end
%             end
%             c1 = c1/numc;
%             c2 = c2/numc;
            %hold on;
            %plot(c1,c2, 'r+', 'MarkerSize', 30, 'LineWidth', 2);
            num = numel(stats);
            num
            if num > 50
                %position = 320;
            end

           
%             if id ~= -1
%                 rectangle('Position', stats(id).BoundingBox, ...
%                 'Linewidth', 3, 'EdgeColor', 'r', 'LineStyle', '--');
%                 bb = stats(id);
%                 
%                 if true%~isnan(c1)
%                     if true%Sensor.cen_in_rect(bb,c1,c2)
%                         %position = bb.BoundingBox(1) + bb.BoundingBox(3)/2.0;
%                         line([position,position],[0,480]);
%                     end
%                 end              
%             end
                        
%             for i = 1:numel(stats)
%                 rectangle('Position', stats(i).BoundingBox, ...
%                 'Linewidth', 1, 'EdgeColor', 'b', 'LineStyle', '--');
%             end
            
            
        end
        
        function position = spot_doors()
            position = -1;
            I = Sensor.get_camera_img();
            imshow(I);
            grayI = rgb2gray(I);        
            BW1 = edge(grayI,'Canny', 0.08);

            [H, T, R] = hough(BW1);
            P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
            lines = houghlines(BW1,T,R,P,'FillGap',40,'MinLength',5);
            imshow(I), hold on
            max_len = 0;
            xy_long = 0;
            for k = 1:length(lines)
                xy=[lines(k).point1; lines(k).point2];
                d = xy(2,2) - xy(1,2);
                if d > 50
               
                    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green')

                    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow')
                    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red')

                    len = norm(lines(k).point1-lines(k).point2);
                    if (len > max_len)
                        max_len = len;
                        xy_long = xy;
                    end
                end
            end

            %door recognition

        end
        
        
    end
end

