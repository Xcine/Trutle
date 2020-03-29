classdef Actuator
    %ACTUATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        v_robot = rospublisher('/mobile_base/commands/velocity');
    end
    
    methods(Static)
        function obj = Actuator()
            %ACTUATOR Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function  walk_step(velocity)    
            velmsg = rosmessage(Actuator.v_robot);
            velmsg.Linear.X = velocity;
            velmsg.Angular.Z = 0;
            send(Actuator.v_robot,velmsg);
            Actuator.eat_candy_if_near();
            
            Sensor.show_camera();
            Sensor.show_laser_scan();
            
            velmsg.Linear.X = 0;
            %disp("Walking ended.")
        end
        
        function  walk_dist(dist,velocity)    
            velmsg = rosmessage(Actuator.v_robot);
            velmsg.Linear.X = velocity;
            velmsg.Angular.Z = 0;
            
            Sensor.show_camera();
            Sensor.show_laser_scan();
            
            [x_0,y_0] = Sensor.get_robot_position();
            cur_dist = 0;
            while cur_dist < dist
                send(Actuator.v_robot,velmsg);
                Actuator.eat_candy_if_near();
                [x_cur,y_cur] = Sensor.get_robot_position();
                cur_dist = sqrt((x_0-x_cur)^2 + (y_0-y_cur)^2);
                %fprintf('cur dist: %f.\n', cur_dist);
                Sensor.show_camera();
                Sensor.show_laser_scan();
            end
            fprintf('walked dist: %f.\n', cur_dist);
            
            velmsg.Linear.X = 0;
            %disp("Walking ended.")
        end
        
        function  walk_time(tics, velocity)    
            velmsg = rosmessage(Actuator.v_robot);
            velmsg.Linear.X = velocity;
            velmsg.Angular.Z = 0;
           
            tic;
            while toc < tics
                send(v_robot,velmsg);
                Sensor.show_camera();
                Sensor.show_laser_scan();
            end
            %disp("Walking ended.");
        end
        
        function rotate_step(velocity)
            %vel+: left rotation, vel-: right rotation
            velmsg = rosmessage(Actuator.v_robot);
            velmsg.Angular.Z = velocity;
            velmsg.Linear.X = 0;
            send(Actuator.v_robot,velmsg);  
            
            Sensor.show_camera();
            Sensor.show_laser_scan();
            
            velmsg.Angular.Z = 0.0;
        end
        
        function rotate_phi(goal_phi, velocity)
            %read first phi
            if velocity < 0
                fac = -1.0;
            else
                fac = 1.0;
            end
            phi_first = Sensor.get_robot_phi()-fac*1;
            %fprintf('first phi: %f.\n',phi_first);
            phi_dif = 0.0;
            tic;
            while phi_dif < goal_phi
                %read actual phi
                phi = Sensor.get_robot_phi();
                %fprintf('phi: %f.\n',phi);
                if toc < 5
                    phi_dif = Sensor.transformPhi(fac*(phi - phi_first), false);
                else
                    phi_dif = Sensor.transformPhi(fac*(phi - phi_first), true);
                end
                %fprintf('phi dif: %f.\n',phi_dif);

                %give spin vel
                Actuator.rotate_step(velocity);
            end
            fprintf('rotated phi: %f.\n',phi_dif);
        end
        
        function rotate_to_global_phi(phi)
            %read first phi
            phi_first = Sensor.get_robot_phi()-1;
            fprintf('first phi: %f.\n',phi_first);

            while abs(phi_first - phi) > 80
                %read actual phi
                phi = Sensor.get_robot_phi();
                fprintf('phi: %f.\n',phi);

                if phi_first - phi < 0 
                    direction = -1.0;
                else
                    direction = 1.0;
                end

                %rotate
                Actuator.rotate_step(0.8*direction);
            end
        end
        
        function rotate_to_circle_rad(rad)
            max_radii = 0;
            while (abs(max_radii-rad) > 3) && (max_radii < rad) 
                [pos,max_radii] = Sensor.get_max_pos_and_rad_in_img();
                
                %give spin vel
                Actuator.rotate_step(0.8);
            end
            fprintf('rotated to rad: %f.\n', max_radii);
        end
        
        function max_phi = get_phi_of_nearest_candy_of_360()
            max_phi = -1;
            max_radii = -1;

            %read first phi
            phi_first = Sensor.get_robot_phi()-1;
            %fprintf('first phi: %f.\n',phi_first);
            phi_dif = 0.0;
            
            tic;
            while phi_dif < 360.0
                %read actual phi
                phi = Sensor.get_robot_phi();
                %fprintf('phi: %f.\n',phi);
                if toc < 5
                    phi_dif = Sensor.transformPhi(phi - phi_first, false);
                else
                    phi_dif = Sensor.transformPhi(phi - phi_first, true);
                end
                %fprintf('phi dif: %f.\n',phi_dif);

                %get shapes
                [cur_pos,cur_radii] = Sensor.get_max_pos_and_rad_in_img();
                if cur_radii > max_radii
                    max_radii = cur_radii;
                    max_phi = phi;
                end

                %give spin vel
                Actuator.rotate_step(0.8);
            end
            fprintf('rotated %f deg and found max phi %f.\n',phi_dif, max_phi);
        end
        
        function max_radii = get_rad_of_nearest_candy_of_360()
            max_phi = -1;
            max_radii = -1;

            %read first phi
            phi_first = Sensor.get_robot_phi()-1;
            %fprintf('first phi: %f.\n',phi_first);
            phi_dif = 0.0;
            
            tic;
            while phi_dif < 360.0
                %read actual phi
                phi = Sensor.get_robot_phi();
                %fprintf('phi: %f.\n',phi);
                if toc < 5
                    phi_dif = Sensor.transformPhi(phi - phi_first, false);
                else
                    phi_dif = Sensor.transformPhi(phi - phi_first, true);
                end
                %fprintf('phi dif: %f.\n',phi_dif);

                %get shapes
                [cur_pos,cur_radii] = Sensor.get_max_pos_and_rad_in_img();
                if cur_radii > max_radii
                    max_radii = cur_radii;
                    max_phi = phi;
                end

                %give spin vel
                Actuator.rotate_step(0.8);
            end
            fprintf('rotated %f deg and found max rad %f.\n',phi_dif, max_radii);
        end
        
        function center_on_circle(from_all)
            if from_all == true
                %phi = select_candy_from_all();
                %rotate2phi(phi);
                rad = Actuator.get_rad_of_nearest_candy_of_360();
                Actuator.rotate_to_circle_rad(rad);
            end

            position = Sensor.get_position_of_near_candy_in_img();

            while ~((position > 310) && (position < 330))
                if position < 320
                    dir = 1.0; %turn left
                elseif position > 320
                    dir = -1.0; %turn right
                else
                    dir = 1,0;
                end
                if abs(position-320.0) < 20
                    Actuator.rotate_step(dir*0.05);
                elseif abs(position-320.0) < 40
                    Actuator.rotate_step(dir*0.05);
                elseif abs(position-320.0) < 70
                    Actuator.rotate_step(dir*0.05);
                else
                    Actuator.rotate_step(dir*0.2);
                end
                position = Sensor.get_position_of_near_candy_in_img();
            end

            line([position,position],[0,480]);
            disp("Centered on Cirlce.");

        end
        
        function center_on_rect()

            position = Sensor.get_position_of_rect();

            while ~((position > 310) && (position < 330))
                if position < 320
                    dir = 1.0; %turn left
                elseif position > 320
                    dir = -1.0; %turn right
                else
                    dir = 1,0;
                end
                if abs(position-320.0) < 20
                    Actuator.rotate_step(dir*0.05);
                elseif abs(position-320.0) < 40
                    Actuator.rotate_step(dir*0.05);
                elseif abs(position-320.0) < 70
                    Actuator.rotate_step(dir*0.05);
                else
                    Actuator.rotate_step(dir*0.2);
                end
                position = Sensor.get_position_of_rect();
            end

            line([position,position],[0,480]);
            disp("Centered on Rectangle.");

        end
        
        function timeout = center_on_door()
            tic;
            position = Sensor.door_in_laser();
            timeout = false;
            while ~((position > 310) && (position < 330))
                if position < 320
                    dir = 1.0; %turn left
                elseif position > 320
                    dir = -1.0; %turn right
                else
                    dir = 1,0;
                end
                if abs(position-320.0) < 20
                    Actuator.rotate_step(dir*0.03);
                elseif abs(position-320.0) < 40
                    Actuator.rotate_step(dir*0.03);
                elseif abs(position-320.0) < 70
                    Actuator.rotate_step(dir*0.07);
                else
                    Actuator.rotate_step(dir*0.3);
                end
                position = Sensor.door_in_laser();
                if toc >20
                    timeout = false;
                end
            end

            line([position,position],[0,480]);
            disp("Centered on Door.");

        end
        
        function bool = walk_through_door_if_there()
            bool = false;
            timeout = Actuator.center_on_door();
            
            while timeout == true
                disp("Time out!");
                Actuator.walk_step(0.5);
                timeout = centeredActuator.center_on_door();
            end
            
            [start_nan,end_nan,ranges] = Sensor.after_door_nans();
            %start_nan,end_nan,ranges
            dist_points = 10;
            alpha = dist_points*0.001636688830331;
            c = ranges(start_nan-1);
            b = ranges(start_nan - dist_points-1);
            c2 = ranges(end_nan+1);
            b2 = ranges(end_nan + dist_points+1);
            dif_b = abs(b2-b);
            fprintf("dif b-b2: %f\n",abs(b2-b));
            if b < b2
                %door on left
                disp("door on left");
                fac = 1.0;
                degs = 90;
            else
                %door on right
                disp("door on right");
                fac = -1.0;
                degs = 90;
                c = ranges(end_nan+1);
                b = ranges(end_nan + dist_points+1);
            end
            
            [beta,y,x] = Sensor.get_beta_y_to_parallel_door(alpha,c,b);
            if x < 4 && y < 4
                beta_deg = (beta*360)/(2*pi);
                fprintf("beta,y,x: %f, %f, %f\n",beta_deg,y,x);
                if beta_deg > 30 && y > 0.5 && dif_b > 0.08
                    Actuator.rotate_phi(beta_deg,fac*0.15);
                    Actuator.walk_dist(y,0.25);
                    Actuator.rotate_phi(degs,-0.25*fac);
                else
                    x = c +0.3;
                end
                Actuator.center_on_door();
                Actuator.walk_step(0.1)
                time_out = Actuator.center_on_door();
                
                while timeout == true
                    disp("Time out!");
                    Actuator.walk_step(0.5);
                    timeout = centeredActuator.center_on_door();
                end
                
                Actuator.walk_dist(x+0.3,0.25);
                bool = true;
                disp("Walked through door");
            else               
                disp("No door in sight, walking random");
                Actuator.walk_dist(3,0.25);    
                bool = false;
            end
        end
        
        function walk_through_door()
            bool = false;
            while bool == false
                bool = Actuator.walk_through_door_if_there();
            end
        end
        
        function eaten = eat_candy_if_near()
            [min_dist, name] = Sensor.get_dist_and_name_of_nearest_candy();
            eaten = false;
            while min_dist < 0.3
                removeModel(Sensor.gazebo,name);
                fprintf("Eaten %s at dist %f.\n", name, min_dist);
                eaten = true;
                [min_dist, name] = Sensor.get_dist_and_name_of_nearest_candy();
            end
        end
        
        function eaten = eat_near_candy()
            Actuator.center_on_circle(false);
            eaten = Actuator.eat_candy_if_near();
            while eaten == false
                eaten = Actuator.eat_candy_if_near();
                Actuator.walk_step(0.2);
            end
        end
        
        function eat_as_much_as_possible()
            tic;
            while toc < 100
                Actuator.eat_near_candy();
            end
            
        end
        
        function eat_number_of_candy(num)
            cur_num = 0;
            while cur_num < num
                eaten = Actuator.eat_near_candy();
                if eaten == true
                    cur_num = cur_num + 1;
                end
            end
            
        end

    end
end

