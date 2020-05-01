classdef Fuzzy_Stop < simiam.controller.Controller
    
    properties
        p
    end 
    
    properties (Constant)
        % I/O
        inputs = struct();
        outputs = struct('wl', 0, 'wr', 0);
    end
    
    methods
        
        function obj = Fuzzy_Stop()
            obj = obj@simiam.controller.Controller('fuzzy_stop');
        end
    end
    
    methods
        
        function outputs = execute(obj, robot, state_estimate, inputs, dt)
            outputs = obj.outputs;
            outputs.wl = 0;
            outputs.wr = 0;
        end
        
    end
    
end