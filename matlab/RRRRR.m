classdef RRRRR
    %5 linkage class
    
    properties (Access = public)
        r % linkage length [r1;r2;r3]
        theta %[theta1;theta2]
        A1 %A1 coordinate [-r3; 0]
        A2 %A2 coordinate [r3; 0]
        b1 %B1 coordinate dependent on theta1
        b2 %B2 coordinate dependent on theta2
        fk_nSol;  %forward kinematics solution number;
        fk_up;    %up-configuration coordinate[x;y]
        fk_down;  %down-configuration coordinate[x;y]
        current_configuration;     %-1 no solution; 0 down; 1 up
        initial_configuration;     % 0 down; 1 up
        current_position;          % end-point coordinate [x; y]
        ik_pp; % ++mode [theta1; theta2]
        ik_pn; % +-mode [theta1; theta2]
        ik_np; % -+mode [theta1; theta2]
        ik_nn; % --mode [theta1; theta2]
    end
    
    methods
        function crossChar = cross2char(~, crossValue)
            %Convert cross product value to '+' '-'
            %Colinear situation is considered as '+'
            if (crossValue < -1E-6)
                crossChar = '-';
            else
                crossChar = '+';
            end
        end
        
        function mode = getWorkMode(fiveLinkage, configuration)
            %After forward kinematics, get work mode
            %input argument:
            %   fiveLinkage: RRRRR class object
            %   configuration: 0 down ; 1 up
            %return
            %   mode:   'xx' no forward kinematics solution
            %           '++' '+-' '-+' '--' corresponding work mode of
            %           specified configuration
            if (fiveLinkage.fk_nSol < 1)
                mode = 'xx';
                return;
            end
            
            p = fiveLinkage.fk_up;
            if (configuration ~= 1)
                p = fiveLinkage.fk_down;
            end
            
            A1C1 = p - fiveLinkage.A1; %vector A1P
            A1B1 = fiveLinkage.b1 - fiveLinkage.A1;    %vector A1B1
            leftCross = A1C1(1) * A1B1(2) - A1B1(1) * A1C1(2);
            
            A2C1 = p  - fiveLinkage.A2;
            A2B2 = fiveLinkage.b2 - fiveLinkage.A2;
            rightCross = A2C1(1) * A2B2(2) - A2B2(1) * A2C1(2);
            
            mode = [fiveLinkage.cross2char(leftCross) , fiveLinkage.cross2char(rightCross)];
            return;
        end
        
        function configuration = getConfiguration(fiveLinkage, mode)
            %Get up/down configuration of specified mode base on current
            %position
            %input argument:
            %   fiveLinkage: RRRRR object
            %   mode: one of  '++' '+-' '-+' '--'
            %return:
            %   -1 no inverse kinematics solution
            %   0 down-configuration
            %   1 up-configuration
            if (isempty(fiveLinkage.current_position))
                configuration = -1;
                return;
            end

            switch mode
                case '++'
                    ik_solution = fiveLinkage.ik_pp;
                case '+-'
                    ik_solution = fiveLinkage.ik_pn;
                case '-+'
                    ik_solution = fiveLinkage.ik_np;
                case '--'
                    ik_solution = fiveLinkage.ik_nn;
                otherwise
                    ik_solution = [];
            end
            
            if (isempty(ik_solution))
                configuration = -1;
                return;
            end
            
            %compute B1 B2
%             B1 = [fiveLinkage.r(1) * cosd(ik_solution(1)) - fiveLinkage.r(3); fiveLinkage.r(1) * sind(ik_solution(1))];
%             B2 = [fiveLinkage.r(1) * cosd(ik_solution(2)) + fiveLinkage.r(3); fiveLinkage.r(1) * sind(ik_solution(2))];
            B1y = fiveLinkage.r(1) * sind(ik_solution(1));
            B2y = fiveLinkage.r(1) * sind(ik_solution(2));
            midY = (B1y + B2y) / 2;
%             p = fiveLinkage.current_position;
%             
%             u = B1 - p; %u is the vector PB1
%             v = B2 - p; %v is the vector PB2
%             cross = u(1) * v(2) - u(2) * v(1);
%             
%             if (cross < -1E-6)
%                 configuration = 0; %down
%             else
%                 configuration = 1;%up
%             end
            
            if (fiveLinkage.current_position(2) >= midY)
                configuration = 1;%up
            else
                configuration = 0; %down
            end

            return;
        end
    end
end
