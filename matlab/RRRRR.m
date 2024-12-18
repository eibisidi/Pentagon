classdef RRRRR
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        r % [r1;r2;r3]
        theta %[theta1;theta2]
        A1 %A1 coordinate [-r3; 0]
        A2 %A2 coordinate [r3; 0]
        b1 %B1 coordinate
        b2 %B2 coordinate
%         h_a1b1; %matlab plot handle of active link A1B1
%         h_a2b2; %matlab plot handle of active link A2B2
%         h_b1c1b2; %matlab plot handle of up-configuration
%         h_b1c2b2; %matlab plot handle of down-configuration
        fk_nSol;  %forward kinematics solution number;
        fk_up;
        fk_down;
        use_up; %current configuration 
        current_configuration;     % 0 down 1 up
        initial_configuration;     % 0 down 1 up
        current_position;
    end
    
    methods
        function obj = untitled(inputArg1,inputArg2)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

