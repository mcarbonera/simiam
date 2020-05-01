classdef AvoidObstacles < simiam.controller.Controller

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software

    properties
        
        % memory banks
        E_k
        e_k_1
        
        % gains
        Kp
        Ki
        Kd
        
        % plot support
        p

        % sensor geometry
        calibrated
        sensor_placement
    end
    
    properties (Constant)
        inputs = struct('v', 0);
        outputs = struct('v', 0, 'w', 0)
    end
    
    methods
        
        function obj = AvoidObstacles()
            obj = obj@simiam.controller.Controller('avoid_obstacles');            
            obj.calibrated = false;
            
            obj.Kp = 4;
            obj.Ki = 0.01;
            obj.Kd = 0.01;
            
            obj.E_k = 0;
            obj.e_k_1 = 0;
        end
        
        function outputs = execute(obj, robot, state_estimate, inputs, dt)
            
            % Compute the placement of the sensors
            if(~obj.calibrated)
                obj.set_sensor_geometry(robot);
            end
            
            % Unpack state estimate
            [x, y, theta] = state_estimate.unpack();
            
            % Poll the current IR sensor values 1-5
            ir_distances = robot.get_ir_distances();
                        
            % Interpret the IR sensor measurements geometrically
            ir_distances_wf = obj.apply_sensor_geometry(ir_distances, state_estimate);            
            
            % 1. Compute the heading vector for obstacle avoidance
            
            sensor_gains = [0.7 2 1.2 2 0.7];
            u_i = (ir_distances_wf-repmat([x;y],1,5))*diag(sensor_gains);
            
            K_vet_cont = sensor_gains(3);
            vetSensores = sum(u_i,2);
            vetEquilibrio = [cos(theta) -sin(theta); sin(theta) cos(theta)]*[-0.7*sqrt(2);0]*K_vet_cont;
            
            u_ao = vetSensores + vetEquilibrio;
            soma_ir_rf_x = sensor_gains(1)*(ir_distances(5) - ir_distances(1)) + sensor_gains(2)*(ir_distances(4) - ir_distances(2));
            %fprintf('soma: %f - [%f,%f,%f,%f,%f]\n',soma_ir_rf_x, ir_distances(1), ir_distances(2), ir_distances(3), ir_distances(4), ir_distances(5));
            if(ir_distances(3) < 0.25) % d_unsafe
                %fprintf('[%f,%f]\n[%f,%f]\n[%f,%f]\n',u_ao(1),u_ao(2),vetSensores(1),vetSensores(2),vetEquilibrio(1),vetEquilibrio(2));
                v = 0;
                if(soma_ir_rf_x > 0)
                    u_ao = [cos(pi/2) sin(pi/2); -sin(pi/2) cos(pi/2)] * vetSensores;
                else
                    u_ao = [cos(pi/2) -sin(pi/2); sin(pi/2) cos(pi/2)] * vetSensores;
                end
            else
                v = inputs.v;
            end
            
            %if(abs(((u_ao(1)*cos(theta) + u_ao(2)*sin(theta)) / norm(u_ao)) + 1) < 0.001)
            %end
            %abs(((u_ao(1)*vetSensores(1) + u_ao(2)*vetSensores(2))/(norm(u_ao)*norm(vetSensores))) -1)
            %if(u_ao(1)*vetSensores(1) + u_ao(2)*vetSensores(2))/(norm(u_ao)*norm(vetSensores)) - 1 < 0.01
            %    % Se vetor está na direção de theta
            %    vetEquilibrio = [cos(theta) -sin(theta); sin(theta) cos(theta)]*[-5*sqrt(2);0]*K_vet_cont;
            %    v = inputs.v/2;
            %else
            %    vetEquilibrio = [cos(theta) -sin(theta); sin(theta) cos(theta)]*[-0.7*sqrt(2);0]*K_vet_cont;
            %    v = inputs.v;
            %end
            %v = inputs.v;
            
            % 2. Compute the heading and error for the PID controller
            theta_ao = atan2(u_ao(2),u_ao(1));
            e_k = theta_ao-theta;
            e_k = atan2(sin(e_k),cos(e_k));
            
            e_P = e_k;
            e_I = obj.E_k + e_k*dt;
            e_D = (e_k-obj.e_k_1)/dt;
              
            % PID control on w
            w = obj.Kp*e_P + obj.Ki*e_I + obj.Kd*e_D;
            
            % Save errors for next time step
            obj.E_k = e_I;
            obj.e_k_1 = e_k;
                        
            % plot  
            obj.p.plot_2d_ref(dt, theta, theta_ao, 'g');
                        
%             fprintf('(v,w) = (%0.4g,%0.4g)\n', v,w);

            outputs.v = v;
            outputs.w = w;
        end
        
        % Helper functions
        
        function ir_distances_wf = apply_sensor_geometry(obj, ir_distances, state_estimate)
                    
            % 1. Apply the transformation to robot frame.
            
            ir_distances_rf = zeros(3,5);
            for i=1:5
                x_s = obj.sensor_placement(1,i);
                y_s = obj.sensor_placement(2,i);
                theta_s = obj.sensor_placement(3,i);
                
                R = obj.get_transformation_matrix(x_s,y_s,theta_s);
                ir_distances_rf(:,i) = R*[ir_distances(i); 0; 1];
            end
            
            % 2. Apply the transformation to world frame.
            
            [x,y,theta] = state_estimate.unpack();
            
            R = obj.get_transformation_matrix(x,y,theta);
            ir_distances_wf = R*ir_distances_rf;
            
            ir_distances_wf = ir_distances_wf(1:2,:);
        end
        
        function set_sensor_geometry(obj, robot)
            obj.sensor_placement = zeros(3,5);
            for i=1:5
                [x, y, theta] = robot.ir_array(i).location.unpack();
                obj.sensor_placement(:,i) = [x; y; theta];
            end                        
            obj.calibrated = true;
        end
        
        function R = get_transformation_matrix(obj, x, y, theta)
            R = [cos(theta) -sin(theta) x; sin(theta) cos(theta) y; 0 0 1];
        end
        
        function reset(obj)
            % Reset accumulated and previous error
            obj.E_k = 0;
            obj.e_k_1 = 0;
        end
        
    end
    
end

