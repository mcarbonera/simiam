classdef Simulator < handle
%% SIMULATOR is responsible for stepping the program through the simulation.
%
% Simulator Properties:
%   parent          - AppWindow graphics handle
%   clock           - Global timer for the simulation
%   time_step       - Time step for the simulation
%   split           - Split between calls to step()
%
% Simulator Methods:
%   step            - Executes one time step of the simulation.
%   start           - Starts the simulation.
%   stop            - Stops the simulation.

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software

    properties
        %% PROPERTIES
        
        parent          % AppWindow graphics handle
        clock           % Global timer for the simulation
        time_step       % Time step for the simulation
        
        world           % A virtual world for the simulator
        physics
        
        origin
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
        % Video
        vobj
        gravarvideo
        
        % gif
        gravargif
        n
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    methods
        %% METHODS
        
        function obj = Simulator(parent, world, time_step, origin)
        %% SIMULATOR Constructor
        %   obj = Simulator(parent, time_step) is the default constructor
        %   that sets the graphics handle and the time step for the
        %   simulation.
        
            obj.parent = parent;
            obj.time_step = time_step;
            if(strcmp(origin, 'launcher'))
                obj.clock = timer('Period', obj.time_step, ...
                                  'TimerFcn', @obj.step, ...
                                  'ExecutionMode', 'fixedRate', ...
                                  'StartDelay', obj.time_step);
            else
                obj.clock = [];
            end
            obj.world = world;
            obj.physics = simiam.simulator.Physics(world);
            obj.origin = origin;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % gif:
            % https://www.mathworks.com/matlabcentral/answers/94495-how-can-i-create-animated-gif-images-in-matlab
            obj.gravarvideo = false;
            obj.gravargif = false;
            
            if(obj.gravarvideo)
                obj.vobj=VideoWriter('MyMovieFile', 'Motion JPEG AVI');
                obj.vobj.FrameRate=60;
                obj.vobj.Quality=100;
                open(obj.vobj);
            elseif(obj.gravargif)
                obj.vobj = 'testAnimated.gif';
                obj.n = 1;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        function step(obj, src, event)
        %% STEP Executes one time step of the simulation.
        %   step(obj, src, event) is the timer callback which is executed
        %   once every time_step seconds.
        
%             if(strcmp(obj.origin, 'launcher'))
%                 split = obj.time_step;
%             else
%                 split = max(obj.time_step,get(obj.clock, 'InstantPeriod'));
%             end

            split = obj.time_step;
            fprintf('***TIMING***\nsimulator split: %0.3fs, %0.3fHz\n', split, 1/split);
            
%             tstart = tic;
            nRobots = length(obj.world.robots);
            for k = 1:nRobots
                robot_s = obj.world.robots.elementAt(k);
                
                [x, y, theta] = robot_s.robot.update_state(robot_s.pose, split).unpack();
                robot_s.pose.set_pose([x, y, theta]);
                fprintf('current_pose: (%0.3f,%0.3f,%0.3f)\n', x, y, theta);
                
                robot_s.supervisor.execute(split);
            end
%             fprintf('controls: %0.3fs\n', toc(tstart));
            
%             tstart = tic;
            if strcmp(obj.origin, 'simulink')
                % skip
            else
                anApp = obj.world.apps.elementAt(1);
                anApp.run(split);
            end
%             fprintf('app: %0.3fs\n', toc(tstart));
            
%             tstart = tic;
%             if(~obj.islinked)
                bool = obj.physics.apply_physics();
%             else
%                 bool = false;
%             end
%             fprintf('physics: %0.3fs\n', toc(tstart));
            
%             tstart = tic;
            obj.parent.ui_update(split, bool);
            drawnow;
%             fprintf('ui: %0.3fs\n', toc(tstart));

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if(obj.gravarvideo)
                F=getframe(obj.parent.view_);
                writeVideo(obj.vobj, F);
                %cla(gca)
            elseif(obj.gravargif)
                % Capture the plot as an image 
                frame = getframe(obj.parent.view_); 
                im = frame2im(frame); 
                [imind,cm] = rgb2ind(im,256); 
                % Write to the GIF File 
                if obj.n == 1 
                    imwrite(imind,cm,obj.vobj,'gif', 'DelayTime', 0.02, 'Loopcount',inf); 
                    obj.n = obj.n + 1;
                else 
                    imwrite(imind,cm,obj.vobj,'gif', 'DelayTime', 0.02, 'WriteMode','append'); 
                end 
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        function start(obj)
        %% START Starts the simulation.
            if strcmp(obj.origin, 'launcher');
                start(obj.clock);
            end
        end
        
        function stop(obj)
        %% STOP Stops the simulation.
            if strcmp(obj.origin, 'launcher')
                stop(obj.clock);
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if(obj.gravarvideo)
                close(obj.vobj)
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        function shutdown(obj)
            if strcmp(obj.origin, 'launcher')
                obj.stop();
                delete(obj.clock);
            end
        end
    end
    
end
