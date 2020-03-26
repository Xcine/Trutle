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
            
            Sensor.show_camera();
            
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
            
            velmsg.Angular.Z = 0.0;
        end
        
        function rotate_phi(goal_phi, velocity)
            %read first phi
            phi_first = Sensor.get_robot_phi()-1;
            fprintf('first phi: %f.\n',phi_first);
            phi_dif = 0.0;
            tic;
            while phi_dif < goal_phi
                %read actual phi
                phi = Sensor.get_robot_phi();
                fprintf('phi: %f.\n',phi);
                if toc < 5
                    phi_dif = Sensor.transformPhi(phi - phi_first, false);
                else
                    phi_dif = Sensor.transformPhi(phi - phi_first, true);
                end
                fprintf('phi dif: %f.\n',phi_dif);

                %give spin vel
                Actuator.rotate_step(velocity);
            end
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
            fprintf('first phi: %f.\n',phi_first);
            phi_dif = 0.0;
            
            tic;
            while phi_dif < 360.0
                %read actual phi
                phi = Sensor.get_robot_phi();
                fprintf('phi: %f.\n',phi);
                if toc < 5
                    phi_dif = Sensor.transformPhi(phi - phi_first, false);
                else
                    phi_dif = Sensor.transformPhi(phi - phi_first, true);
                end
                fprintf('phi dif: %f.\n',phi_dif);

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
            fprintf('first phi: %f.\n',phi_first);
            phi_dif = 0.0;
            
            tic;
            while phi_dif < 360.0
                %read actual phi
                phi = Sensor.get_robot_phi();
                fprintf('phi: %f.\n',phi);
                if toc < 5
                    phi_dif = Sensor.transformPhi(phi - phi_first, false);
                else
                    phi_dif = Sensor.transformPhi(phi - phi_first, true);
                end
                fprintf('phi dif: %f.\n',phi_dif);

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

    end
end

