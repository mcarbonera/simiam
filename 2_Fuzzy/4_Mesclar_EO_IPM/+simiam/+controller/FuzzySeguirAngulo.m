classdef FuzzySeguirAngulo < simiam.controller.Controller
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
        inputs = struct('theta_d', 0, 'v', 0);
        outputs = struct('wl', 0, 'wr', 0);
    end
    
    methods
    %% METHODS
        
        function obj = FuzzySeguirAngulo()
            %% FuzzySeguirAngulo Constructor
            obj = obj@simiam.controller.Controller('fuzzy_seguir_angulo');
            
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
        
            % Retrieve the (relative) angle location
            theta_d = inputs.theta_d;
            
            % Get estimate of current pose
            [x, y, theta] = state_estimate.unpack();
            
            % Compute the v,w that will get you to the angle
            v = inputs.v;
            
            % heading error
            e_k = theta_d-theta;
            e_k = atan2(sin(e_k), cos(e_k));
            
            % Direction for heading
            %w = obj.Kp*e_k;
            VetX = cos(e_k);
            VetY = sin(e_k);
            
            %FIS Evaluate
            w = evalfis([VetX VetY v],obj.SeguirVetor);
            wl = w(1);
            wr = w(2);
            
            % wl e wr estão no intervalo [0 1]. Multiplicar por velocidade
            % máxima:
            wl = wl * robot.max_vel * obj.K_v;
            wr = wr * robot.max_vel * obj.K_v;
            
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

