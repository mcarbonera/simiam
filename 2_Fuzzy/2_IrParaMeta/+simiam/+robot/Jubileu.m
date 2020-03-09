classdef Jubileu < simiam.robot.Robot

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software
    
    properties
        wheel_radius
        wheel_base_length
        ticks_per_rev
        max_vel
        min_vel
        
        encoders = simiam.robot.sensor.WheelEncoder.empty(1,0);
        ir_array = simiam.robot.sensor.ProximitySensor.empty(1,0);
        
        dynamics
        prev_ticks
    end
    
    properties (SetAccess = private)
        right_wheel_speed
        left_wheel_speed
    end
    
    methods
        function obj = Jubileu(parent, pose)
           obj = obj@simiam.robot.Robot(parent, pose);
           
           % Add surfaces: Jubileu in top-down 2D view                        
            qb_base_plate = [%
                             0.052         -0.069      1;
                             0.052         -0.102      1;
                             
                             0.1406637      -0.102      1;
                             0.146634       -0.0972876  1;
                             0.1622876      -0.0816340  1;
                             0.1749852      -0.0635     1;
                             0.184341      -0.0434366  1;
                             0.1900706      -0.0220533  1;
                             0.192          0           1;
                             0.1900706      0.0220533   1;
                             0.184341      0.0434366   1;
                             0.1749852      0.0635      1;
                             0.1622876      0.0816340   1;
                             0.146634       0.0972876   1;
                             0.1406637      0.102       1;
                             
                             0.052         0.102       1;
                             0.052         0.069       1;
                             
                             -0.0416208     0.069      1;
                             -0.0449852     0.0635     1;
                             -0.054341     0.0434366  1;
                             -0.0600706     0.0220533  1;
                             -0.062         0           1;
                             -0.0600706     -0.0220533   1;
                             -0.054341     -0.0434366   1;
                             -0.0449852     -0.0635      1;
                             -0.0416208     -0.069       1;
                             ];
                         
            qb_bbb =  [ -0.0914 -0.0406 1;
                        -0.0944 -0.0376 1;
                        -0.0944  0.0376 1;
                        -0.0914  0.0406 1;
                        -0.0429  0.0406 1;
                        -0.0399  0.0376 1;
                        -0.0399 -0.0376 1;
                        -0.0429 -0.0406 1];
                    
            qb_bbb_rail_l = [ -0.0429 -0.0356 1;
                              -0.0429  0.0233 1;
                              -0.0479  0.0233 1;
                              -0.0479 -0.0356 1;];
                          
            qb_bbb_rail_r = [ -0.0914 -0.0356 1;
                              -0.0914  0.0233 1;
                              -0.0864  0.0233 1;
                              -0.0864 -0.0356 1;
                              ];
                          
            qb_bbb_eth = [ -0.0579 0.0436 1;
                           -0.0579 0.0226 1;
                           -0.0739 0.0226 1;
                           -0.0739 0.0436 1;];
                       
            qb_left_wheel = [0.034  0.077   1;
                             -0.034 0.077   1;
                             -0.034 0.104   1;
                             0.034  0.104   1;
                            ];
                          
            qb_left_wheel_ol = [0.034  0.077   1;
                                -0.034 0.077   1;
                                -0.034 0.104   1;
                                0.034  0.104   1;
                               ];
            
            qb_right_wheel_ol = [0.034  -0.104  1;
                                 -0.034 -0.104  1;
                                 -0.034 -0.077  1;
                                 0.034  -0.077  1;
                                ];
                         
            qb_right_wheel = [0.034  -0.104  1;
                              -0.034 -0.104  1;
                              -0.034 -0.077  1;
                              0.034  -0.077  1;
                             ];

            offsetXLat = 0.12;
            offsetXInc = 0.045;
            offsetYInc = 0.005;
            offsetXFron = 0.045;
            qb_ir_1 = [ -0.0732+offsetXLat, 0.0534 1;
                        -0.0732+offsetXLat, 0.0634 1;
                        -0.0432+offsetXLat, 0.0634 1;
                        -0.0432+offsetXLat, 0.0534 1];
                    
            qb_ir_2 = [ 0.0643+offsetXInc 0.0214+offsetYInc 1;
                        0.0714+offsetXInc 0.0285+offsetYInc 1;
                        0.0502+offsetXInc 0.0497+offsetYInc 1;
                        0.0431+offsetXInc 0.0426+offsetYInc 1];
                        
                    
            qb_ir_3 = [ 0.0636+offsetXFron -0.0042 1;
                        0.0636+offsetXFron  0.0258 1;
                        0.0736+offsetXFron  0.0258 1;
                        0.0736+offsetXFron -0.0042 1];
                    
            qb_ir_4 = [ 0.0643+offsetXInc -0.0214-offsetYInc 1;
                        0.0714+offsetXInc -0.0285-offsetYInc 1;
                        0.0502+offsetXInc -0.0497-offsetYInc 1;
                        0.0431+offsetXInc -0.0426-offsetYInc 1];
                    
            qb_ir_5 = [ -0.0732+offsetXLat, -0.0534 1;
                        -0.0732+offsetXLat, -0.0634 1;
                        -0.0432+offsetXLat, -0.0634 1;
                        -0.0432+offsetXLat, -0.0534 1];

            qb_bbb_usb = [ -0.0824 -0.0418 1;
                           -0.0694 -0.0418 1;
                           -0.0694 -0.0278 1;
                           -0.0824 -0.0278 1];
            
            obj.add_surface_with_depth(qb_base_plate, [ 226 0 2 ]/255, 1.3);
            obj.add_surface(qb_right_wheel, [ 0.15 0.15 0.15 ]);
            s = obj.add_surface_with_depth(qb_right_wheel_ol, [0.15 0.15 0.15], 1.5);
            
            set(s.handle_, 'LineStyle', '--');
            set(s.handle_, 'FaceColor', 'none');
            
            s = obj.add_surface_with_depth(qb_left_wheel_ol, [0.15 0.15 0.15], 1.5);
            
            set(s.handle_, 'LineStyle', '--');
            set(s.handle_, 'FaceColor', 'none');
%            obj.add_surface_with_alpha(qb_axle, [0.15 0.15 0.15], 0.5);
            obj.add_surface(qb_left_wheel, [ 0.15 0.15 0.15 ]);
            
            obj.add_surface_with_depth(qb_ir_1, [0.1 0.1 0.1], 10);
            obj.add_surface_with_depth(qb_ir_2, [0.1 0.1 0.1], 10);
            obj.add_surface_with_depth(qb_ir_3, [0.1 0.1 0.1], 10);
            obj.add_surface_with_depth(qb_ir_4, [0.1 0.1 0.1], 10);
            obj.add_surface_with_depth(qb_ir_5, [0.1 0.1 0.1], 10);
                       
            %obj.add_surface_with_depth(qb_bbb, [0.2 0.2 0.2], 1.4);
            %obj.add_surface_with_depth(qb_bbb_rail_l, [0 0 0], 1.5);
            %obj.add_surface_with_depth(qb_bbb_rail_r, [0 0 0], 1.5);
            %obj.add_surface_with_depth(qb_bbb_eth, [0.7 0.7 0.7], 1.5);
            %obj.add_surface_with_depth(qb_bbb_usb, [0.7 0.7 0.7], 1.5);
                
            % Add sensors: wheel encoders and IR proximity sensors
            obj.wheel_radius = 0.034;           % 65.0mm in diameter
            obj.wheel_base_length = 0.18;     % 99.25mm
            obj.ticks_per_rev = 297;
            
            % Adjust these MAX/MIN RPM values for your Jubileu and surface
            
            max_rpm = 166.5;
            obj.max_vel = max_rpm*2*pi/60; % 1.8266 rad/s
            
            min_rpm = 60;
            obj.min_vel = min_rpm*2*pi/60; % 0.6580 rad/s
            
            % vel_linear_min = 0.21363708953606095 m/s
            % vel_linear_max = 0.593063493182867 m/s
            
            obj.encoders(1) = simiam.robot.sensor.WheelEncoder('right_wheel', obj.wheel_radius, obj.wheel_base_length, obj.ticks_per_rev);
            obj.encoders(2) = simiam.robot.sensor.WheelEncoder('left_wheel', obj.wheel_radius, obj.wheel_base_length, obj.ticks_per_rev);
            
            import simiam.robot.sensor.ProximitySensor;
            import simiam.robot.Jubileu;
            import simiam.ui.Pose2D;
            
            ir_pose = Pose2D(-0.0474+offsetXLat, 0.0534, Pose2D.deg2rad(90));
            obj.ir_array(1) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.1, 0.86, Pose2D.deg2rad(6), 'simiam.robot.Jubileu.ir_distance_to_raw');
            
            ir_pose = Pose2D(0.0613+offsetXInc, 0.0244+offsetYInc, Pose2D.deg2rad(45));
            obj.ir_array(2) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.1, 0.86, Pose2D.deg2rad(6), 'simiam.robot.Jubileu.ir_distance_to_raw');
            
            ir_pose = Pose2D(0.0636+offsetXFron, 0.0, Pose2D.deg2rad(0));
            obj.ir_array(3) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.1, 0.86, Pose2D.deg2rad(6), 'simiam.robot.Jubileu.ir_distance_to_raw');
            
            ir_pose = Pose2D(0.0461+offsetXInc,  -0.0396-offsetYInc, Pose2D.deg2rad(-45));
            obj.ir_array(4) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.1, 0.86, Pose2D.deg2rad(6), 'simiam.robot.Jubileu.ir_distance_to_raw');
            
            ir_pose = Pose2D(-0.0690+offsetXLat, -0.0534, Pose2D.deg2rad(-90));
            obj.ir_array(5) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.1, 0.86, Pose2D.deg2rad(6), 'simiam.robot.Jubileu.ir_distance_to_raw');
            
            % Add dynamics: two-wheel differential drive
            obj.dynamics = simiam.robot.dynamics.DifferentialDrive(obj.wheel_radius, obj.wheel_base_length);
            
            obj.right_wheel_speed = 0;
            obj.left_wheel_speed = 0;
            
            obj.prev_ticks = struct('left', 0, 'right', 0);
        end
        
        function ir_distances = get_ir_distances(obj)
            ir_array_values = obj.ir_array.get_range();
            
            %% START CODE BLOCK %%
            
            %ir_voltages = ir_array_values*3/1000;
            %coeff = [-0.0182 0.1690 -0.6264 1.1853 -1.2104 0.6293];
            
            ir_voltages = ir_array_values*0.0008058608059;
            a = 2.7802212625047;
            b = -35.1150300110164;
            c = 179.6031433005229;
            d = -477.9449116299037;
            e = 706.3400747125301;
            f = -569.7367375002304;
            g = 221.2678651473199;
            coeff = [a b c d e f g];
            
            %% END CODE BLOCK %%
            
            ir_distances = polyval(coeff, ir_voltages);
        end
        
        % Hardware connectivty related functions
        function add_hardware_link(obj, hostname, port)
            obj.driver = simiam.robot.driver.JubileuDriver(hostname, port);
        end
        
        function pose_new = update_state_from_hardware(obj, pose, dt)
            
            encoder_ticks = obj.driver.get_encoder_ticks();
            
            if (~isempty(encoder_ticks))
                obj.encoders(2).ticks = encoder_ticks(1);
                obj.encoders(1).ticks = encoder_ticks(2);
            end
            
            ir_raw_values = obj.driver.get_ir_raw_values();
            
            if (~isempty(ir_raw_values))
                %ir_voltages = ir_raw_values*3/1000;
                %coeff = [-0.0182 0.1690 -0.6264 1.1853 -1.2104 0.6293];
                ir_voltages = ir_raw_values*0.0008058608059;
                a = 2.7802212625047;
                b = -35.1150300110164;
                c = 179.6031433005229;
                d = -477.9449116299037;
                e = 706.3400747125301;
                f = -569.7367375002304;
                g = 221.2678651473199;
                coeff = [a b c d e f g];
            
                ir_distances = polyval(coeff, ir_voltages);
                
                for i = 1:numel(obj.ir_array)
                    obj.ir_array(i).update_range(ir_distances(i));
                end
            end
            
            right_rps = obj.right_wheel_speed/(2*pi);
            left_rps = obj.left_wheel_speed/(2*pi);
            
            % beta: coeficientes a,b de uma reta y(rps) = ax(%pwm) + b
            beta = [0.022482348354419, 0.527911826540430];
            %beta = [0.0425, -0.9504];
            
            % x(%pwm) = [y(rps) - b]/a 
            right_pwm = sign(right_rps)*(abs(right_rps)-beta(2))/beta(1);
            left_pwm = sign(left_rps)*(abs(left_rps)-beta(2))/beta(1);

            right_pwm = max(min(round(right_pwm), 100), -100);
            left_pwm = max(min(round(left_pwm), 100), -100);
            
            obj.driver.set_speeds(right_pwm, left_pwm);
            
            pose_new = obj.update_pose_from_hardware(pose);
            
            obj.update_pose(pose_new);
            
            for k=1:length(obj.ir_array)
                obj.ir_array(k).update_pose(pose_new);
            end
        end
        
        function pose_k_1 = update_pose_from_hardware(obj, pose)
            right_ticks = obj.encoders(1).ticks;
            left_ticks = obj.encoders(2).ticks;
            
            prev_right_ticks = obj.prev_ticks.right;
            prev_left_ticks = obj.prev_ticks.left;
            
            obj.prev_ticks.right = right_ticks;
            obj.prev_ticks.left = left_ticks;
            
            [x, y, theta] = pose.unpack();
                        
            m_per_tick = (2*pi*obj.wheel_radius)/obj.encoders(1).ticks_per_rev;
            
            d_right = (right_ticks-prev_right_ticks)*m_per_tick;
            d_left = (left_ticks-prev_left_ticks)*m_per_tick;
            
            d_center = (d_right + d_left)/2;
            phi = (d_right - d_left)/obj.wheel_base_length;
            
            theta_new = theta + phi;
            x_new = x + d_center*cos(theta);
            y_new = y + d_center*sin(theta);
                                       
            % Update your estimate of (x,y,theta)
            pose_k_1 = simiam.ui.Pose2D(x_new, y_new, theta_new);
        end
        
        
        function pose = update_state(obj, pose, dt)
            R = obj.wheel_radius;
            
            vel_r = obj.right_wheel_speed;     % mm/s
            vel_l = obj.left_wheel_speed;      % mm/s
            
            pose = obj.dynamics.apply_dynamics(pose, dt, vel_r, vel_l);
            obj.update_pose(pose);
            
            for k=1:length(obj.ir_array)
                obj.ir_array(k).update_pose(pose);
            end
            
            % update wheel encoders
            
            vel_r = obj.right_wheel_speed; %% mm/s
            vel_l = obj.left_wheel_speed; %% mm/s
            
            obj.encoders(1).update_ticks(vel_r, dt);
            obj.encoders(2).update_ticks(vel_l, dt);
        end
        
        function set_wheel_speeds(obj, vel_r, vel_l)
            [vel_r, vel_l] = obj.limit_speeds(vel_r, vel_l);
            R = obj.wheel_radius;
            
            obj.right_wheel_speed = vel_r;
            obj.left_wheel_speed = vel_l;
        end
        
        function [vel_r, vel_l] = limit_speeds(obj, vel_r, vel_l)
            % actuator hardware limits            
            vel_r = max(min(vel_r, obj.max_vel), -obj.max_vel);
            vel_l = max(min(vel_l, obj.max_vel), -obj.max_vel);
            
            vel_r = vel_r*(abs(vel_r) >= obj.min_vel);
            vel_l = vel_l*(abs(vel_l) >= obj.min_vel);
        end
    end
    
    methods (Static)
        function raw = ir_distance_to_raw(varargin)
            distances = cell2mat(varargin);
            nSensors = numel(distances);
            
            %coeff = [-10749.324, 10994.428, -4448.079, 906.570, -98.411, 5.496];
            a = 0.000000000021158;
            b = -0.000000015598650;
            c = 0.000003262045546;
            d = -0.000307797135553;
            e = 0.014873822221472;
            f = -0.375524650834571;
            g = 4.834826879426657;
            coeff = [a, b, c, d, e, f, g];
           
            voltages = zeros(nSensors, 1);
            for i = 1:nSensors
                voltages(i) = polyval(coeff, distances(i)*100);
%                 fprintf('%0.3f, %0.3f\n', distances(i), voltages(i));
            end
                
            %raw = min(max(round(voltages/0.0008058608059), 133), 917);
            raw = min(max(round(voltages/0.0008058608059), 498), 2842);
            % Valores para 80cm e 10cm respectivamente.
            % (rampa com satura��o superior e inferior)
        end
    end
end