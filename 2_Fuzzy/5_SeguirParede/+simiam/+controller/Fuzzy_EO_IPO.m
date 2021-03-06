classdef Fuzzy_EO_IPO < simiam.controller.Controller

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software

    properties
        % memory banks
        % plot support
        p
        
        % FIS - Fuzzy Inference System
        SeguirVetor
        EvitarObstaculo
        
        % Ganho para velocidade m�xima:
        K_v
        
        % IR plot support
        u_arrow
        u_arrow_r
        s_net
        
        % sensor geometry
        calibrated
        sensor_placement
    end
    
    properties (Constant)
        inputs = struct('x_g', 0, 'y_g', 0, 'v', 0);
        outputs = struct('wl', 0, 'wr', 0)
    end
    
    methods
        
        function obj = Fuzzy_EO_IPO()
            obj = obj@simiam.controller.Controller('fuzzy_eo_ipo');            
            obj.calibrated = false;
            
            % Fuzzy Inference System:
            obj.SeguirVetor = readfis('/+simiam/+controller/SeguirVetor.fis');
            obj.EvitarObstaculo = readfis('/+simiam/+controller/EO2.fis');
            % Ganho de velocidade (no intervalo [0 1]) 
            obj.K_v = 1;           
        end
        
        function outputs = execute(obj, robot, state_estimate, inputs, dt)
            
            % Compute the placement of the sensors
            if(~obj.calibrated)
                obj.set_sensor_geometry(robot);
                
                % Temporary plot support
                hold(robot.parent, 'on');
                obj.u_arrow_r = plot(robot.parent, [0 0], [0 0], 'b-x', 'LineWidth', 2);
                obj.u_arrow = plot(robot.parent, [0 0], [0 0], 'r--x', 'LineWidth', 2);
                obj.s_net = plot(robot.parent, zeros(1,5), zeros(1,5), 'kx', 'MarkerSize', 8);
                set(obj.u_arrow_r, 'ZData', ones(1,2));
                set(obj.u_arrow, 'ZData', ones(1,2));
                set(obj.s_net, 'ZData', ones(1,5));
            end
            
            % Retrieve the (relative) goal location:
            x_g = inputs.x_g;
            y_g = inputs.y_g;
            
            % Unpack state estimate
            [x, y, theta] = state_estimate.unpack();
            
            %% Ir Para Objetivo (IPO)
            
            % 1. Calculate the heading (angle) to the goal.
            % distance between goal and robot in x-direction
            u_x = x_g - x;     
            % distance between goal and robot in y-direction
            u_y = y_g - y;
            % Norma:
            u_Mag = norm([u_x u_y]);
            
            % angle from robot to goal. Hint: use ATAN2, u_x, u_y here.
            theta_g = atan2(u_y, u_x);
            
            % 2. Calculate the heading error.
            % error between the goal angle and robot's angle
            
            if u_Mag >= 1
                IPO_VetX = cos(theta_g - theta);
                IPO_VetY = sin(theta_g - theta);
            else
                IPO_VetX = u_Mag*cos(theta_g - theta);
                IPO_VetY = u_Mag*sin(theta_g - theta);
            end
            
            %% Evitar Obst�culo (EO)
            % Poll the current IR sensor values 1-5
            ir_distances = robot.get_ir_distances(); % Entrada para FIS
                     
            % Interpret the IR sensor measurements geometrically
            % Para plotar
            ir_distances_wf = obj.apply_sensor_geometry(ir_distances, state_estimate);            
            
            %% START CODE BLOCK %%
            
            % Compute the heading vector
            
            %n_sensors = length(robot.ir_array);
            %sensor_gains = [1 2 1 2 1];
            %sensor_gains = [1 1 1 1 1];
            
            %u_i = zeros(2,5);
            %for i = 1:size(ir_distances_wf,2)
            %    u_i(:,i) = ir_distances_wf(:,i) - [x;y];
            %    u_i(:,i) = u_i(:,i) * sensor_gains(i);
            %end
            
            % Entrada para Fuzzy Inference System
            dist = zeros(1,5);
            dist(1) = ir_distances(1);
            dist(2) = ir_distances(5);
            dist(3) = ir_distances(2);
            dist(4) = ir_distances(4);
            dist(5) = ir_distances(3);
            
            %u_ao = sum(u_i,2);
            
            %FIS Evaluate
            VetRec = evalfis(dist/100,obj.EvitarObstaculo);
            EO_VetX_SE = VetRec(1);
            EO_VetY_SE = VetRec(2);
            EO_VetX_SD = VetRec(3);
            EO_VetY_SD = VetRec(4);
            EO_VetX_SDE = VetRec(5);
            EO_VetY_SDE = VetRec(6);
            EO_VetX_SDD = VetRec(7);
            EO_VetY_SDD = VetRec(8);
            EO_VetX_SF = VetRec(9);
            EO_VetY_SF = VetRec(10);
            
            EO_VetX = EO_VetX_SDE + EO_VetX_SDD + 2*EO_VetX_SF;
            EO_VetY = EO_VetY_SE + EO_VetY_SD + EO_VetY_SDE + EO_VetY_SDD;
            
            v = VetRec(11);
            
            % Normalizar recomenda��o EO:
            norm_EO_Vet = norm([EO_VetX EO_VetY]);
            if norm_EO_Vet < 0.0001 % ==0
                EO_VetX = EO_VetX/norm_EO_Vet;
                EO_VetY = EO_VetY/norm_EO_Vet;
            end
            
            %ResX = v*IPO_VetX + (1-v)*EO_VetX;
            %ResY = v*IPO_VetY + (1-v)*EO_VetY;
            alpha = 1;
            ResX = IPO_VetX + alpha*EO_VetX;
            ResY = IPO_VetY + alpha*EO_VetY;
            %Res = [ResX ResY];
            if abs(ResX) > 1
                ResY = ResY/abs(ResX);
                ResX = ResX/abs(ResX);
            end
            if abs(ResY) > 1
                ResX = ResX/abs(ResY);
                ResY = ResY/abs(ResY);
            end
            
            IPOVet = [IPO_VetX IPO_VetY];
            EOVet = [EO_VetX EO_VetX];
            Input_SV = [ResX ResY v];
            w = evalfis(Input_SV,obj.SeguirVetor);
            wl = w(1);
            wr = w(2);
            
            % O retorno de velocidade do sistema fuzzy "Seguir Angulo"
            % est� no intervalo [0,1] (normalizado)
            % Os motores do Jubileu possuem zona morta de magnitude
            % consider�vel. Portanto, os valores no intervalo [0,1] foram
            % convertidos para [w_sat_norm, 1], onde w_sat_norm corresponde
            % a um valor normalizado suficiente para evitar a zona morta.
            % Regra de tr�s para o valor encaixar em [w_sat_norm, 1]:
            % 0     ~ w_min
            % w_rec ~ w
            % 1     ~ w_max
            % (w_max-w_min)   ~ 1-0
            % (w - w_min)     ~ (w_rec-0)
            % (w-w_min)*1 = w_rec*(w_max-w_min)
            % w = w_min + w_rec*(w_max-w_min);
            %   qdo w_rec = 0 -> w = w_min
            %   qdo w_rec = 1 -> w = w_max
            wl = (robot.min_vel) + wl*(robot.max_vel - robot.min_vel);
            wr = (robot.min_vel) + wr*(robot.max_vel - robot.min_vel);
            
            % heading - para plotar
            u_ao = [ResX ResY];
            theta_ao = atan2(u_ao(2), u_ao(1));
            %e_k = theta_ao - theta;
            %e_k = atan2(sin(e_k),cos(e_k));
            
            %% END CODE BLOCK %%
                                    
            %e_P = e_k;
            %e_I = obj.E_k + e_k*dt;
            %e_D = (e_k-obj.e_k_1)/dt;
              
            % PID control on w
            %v = inputs.v;
            %w = obj.Kp*e_P + obj.Ki*e_I + obj.Kd*e_D;
            
            % Save errors for next time step
            %obj.E_k = e_I;
            %obj.e_k_1 = e_k;
                        
            % plot
            obj.p.plot_2d_ref(dt, atan2(sin(theta),cos(theta)), theta_ao, 'g');
            
%             fprintf('(v,w) = (%0.4g,%0.4g)\n', v,w);
            
            % Temporary plot support
            %u_n = u_ao/(4*norm(u_ao));
            u_n = u_ao/(norm(u_ao));
            sizeArrow = norm(u_ao);
            set(obj.u_arrow, 'XData', [x x+sizeArrow*(cos(theta+theta_ao))]);
            set(obj.u_arrow, 'YData', [y y+sizeArrow*(sin(theta+theta_ao))]);
            set(obj.u_arrow_r, 'XData', [x x+cos(theta)]);
            set(obj.u_arrow_r, 'YData', [y y+sin(theta)]);
            set(obj.s_net, 'XData', ir_distances_wf(1,:));
            set(obj.s_net, 'YData', ir_distances_wf(2,:));

            % velocity control
            outputs = obj.outputs;  % make a copy of the output struct
            outputs.wl = wl;
            outputs.wr = wr;
        end
        
        % Helper functions
        
        function ir_distances_wf = apply_sensor_geometry(obj, ir_distances, state_estimate)
            n_sensors = numel(ir_distances);
            
            %% START CODE BLOCK %%
            
            % Apply the transformation to robot frame.
            
            ir_distances_rf = zeros(3,n_sensors);
            for i=1:n_sensors
                x_s = obj.sensor_placement(1,i);
                y_s = obj.sensor_placement(2,i);
                theta_s = obj.sensor_placement(3,i);
                
                R = obj.get_transformation_matrix(x_s,y_s,theta_s);
                %ir_distances_rf(:,i) = zeros(3,1);
                ir_distances_rf(:,i) = R*[ir_distances(i);0;1];
            end
            
            % Apply the transformation to world frame.
            
            [x,y,theta] = state_estimate.unpack();
            
            R = obj.get_transformation_matrix(x,y,theta);
            %ir_distances_wf = zeros(3,5);
            ir_distances_wf = R*ir_distances_rf;
            
            %% END CODE BLOCK %%
            
            ir_distances_wf = ir_distances_wf(1:2,:);
        end
        
        
        function R = get_transformation_matrix(obj, x, y, theta)
            %% START CODE BLOCK %%
            %R = zeros(3,3);
            R = [[cos(theta),-sin(theta),x];
                [sin(theta),cos(theta),y];
                [0,0,1]];
            %% END CODE BLOCK %%
        end
        
        function set_sensor_geometry(obj, robot)
            n_sensors = length(robot.ir_array);
            obj.sensor_placement = zeros(3,n_sensors);
            for i=1:n_sensors
                [x, y, theta] = robot.ir_array(i).location.unpack();
                obj.sensor_placement(:,i) = [x; y; theta];
            end                        
            obj.calibrated = true;
        end
        
    end
    
end
