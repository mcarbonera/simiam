classdef FuzzyIrParaMeta < simiam.controller.Controller
%% GOTOANGLE steers the robot towards a angle with a constant velocity using PID
%
% Properties:
%   none
%
% Methods:
%   execute - Computes the left and right wheel speeds for go-to-angle.
    
    properties
        %% PROPERTIES
        
        % memory banks
        
        % plot support
        p
        
        % FIS - Fuzzy Inference System
        SeguirVetor
        
        % Ganho para velocidade máxima:
        K_v
    end
    
    properties (Constant)
        % I/O
        inputs = struct('x_g', 0, 'y_g', 0, 'v', 0);
        outputs = struct('wl', 0, 'wr', 0);
    end
    
    methods
    %% METHODS
        
        function obj = FuzzyIrParaMeta()
            %% FuzzySeguirAngulo Constructor
            obj = obj@simiam.controller.Controller('fuzzy_ipo');
            
            % initialize memory banks
            % Fuzzy Inference System:
            obj.SeguirVetor = readfis('/+simiam/+controller/SeguirVetor.fis');
            % Ganho de velocidade (no intervalo [0 1]) 
            obj.K_v = 1;
            
            % plot support
%             obj.p = simiam.util.Plotter();
        end
        
        function outputs = execute(obj, robot, state_estimate, inputs, dt)
        %% EXECUTE Computes the left and right wheel speeds for go-to-angle.
        %   [v, w] = execute(obj, robot, x_g, y_g, v) will compute the
        %   necessary linear and angular speeds that will steer the robot
        %   to the angle location (x_g, y_g) with a constant linear velocity
        %   of v.
        %
        %   See also controller/execute
        
            % Retrieve the (relative) goal location:
            x_g = inputs.x_g; 
            y_g = inputs.y_g;
            
            % Get estimate of current pose
            [x, y, theta] = state_estimate.unpack();
            
            % Compute the v,w that will get you to the angle
            v = inputs.v;
            
            % 1. Calculate the heading (angle) to the goal.
            % distance between goal and robot in x-direction
            u_x = x_g - x;     
            % distance between goal and robot in y-direction
            u_y = y_g - y;
            
            % angle from robot to goal. Hint: use ATAN2, u_x, u_y here.
            theta_g = atan2(u_y, u_x);
            
            % 2. Calculate the heading error.
            % error between the goal angle and robot's angle
            % Hint: Use ATAN2 to make sure this stays in [-pi,pi].
            VetX = cos(theta_g - theta);
            VetY = sin(theta_g - theta);
            %e_k = atan2(sin(theta_g-theta), cos(theta_g-theta));
            %VetX = u_x/(sqrt(u_x^2 + u_y^2))
            %VetY = u_y/(sqrt(u_x^2 + u_y^2))
            
            %FIS Evaluate
            w = evalfis([VetX VetY v],obj.SeguirVetor);
            wl = w(1);
            wr = w(2);
            
            % O retorno de velocidade do sistema fuzzy "Seguir Angulo"
            % está no intervalo [0,1] (normalizado)
            % Os motores do Jubileu possuem zona morta de magnitude
            % considerável. Portanto, os valores no intervalo [0,1] foram
            % convertidos para [w_sat_norm, 1], onde w_sat_norm corresponde
            % a um valor normalizado suficiente para evitar a zona morta.
            % Regra de três para o valor encaixar em [w_sat_norm, 1]:
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
            
            % plot
%             obj.p.plot_2d_ref(dt, atan2(sin(theta),cos(theta)), theta_d, 'g');
            
            % print IR measured distances
            ir_distances = robot.get_ir_distances();
            for i=1:numel(ir_distances)
%                 fprintf('IR %d: %0.3fm\n', i, ir_distances(i));
            end
            
            outputs = obj.outputs;  % make a copy of the output struct
            outputs.wl = wl;
            outputs.wr = wr;
        end
        
    end
    
end

