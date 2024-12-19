classdef RRRRR
    %basic 5 bar linkage class
    
    properties (Access = public)
        r % linkage length [r1;r2;r3]
        theta %[theta1;theta2]
        A1 %A1 coordinate [-r3; 0]
        A2 %A2 coordinate [r3; 0]
        B1 %B1 coordinate dependent on theta1
        B2 %B2 coordinate dependent on theta2
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
            A1B1 = fiveLinkage.B1 - fiveLinkage.A1;    %vector A1B1
            leftCross = A1C1(1) * A1B1(2) - A1B1(1) * A1C1(2);
            
            A2C1 = p  - fiveLinkage.A2;
            A2B2 = fiveLinkage.B2 - fiveLinkage.A2;
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
            B1y = fiveLinkage.r(1) * sind(ik_solution(1));
            B2y = fiveLinkage.r(1) * sind(ik_solution(2));
            midY = (B1y + B2y) / 2;
            
            if (fiveLinkage.current_position(2) >= midY)
                configuration = 1;%up
            else
                configuration = 0; %down
            end
            
            return;
        end
        
        function [nSol, up, down] = forwardKinematics(fiveLinkage)
            %forward kinematics
            %input arguments:
            %               r: [r1; r2; r3] column vector
            %               theta: [theta1; theta2] column vector
            %return value:
            %           nSol: number of solutions.
            %           	-1: infinitely many solutions, B1 and B2 coincides
            %               0: no solution 1/2
            %               1: up and down is the same point
            %               2: up and down is different
            %           up: up-configuration end point coordinate [x;y]
            %           down: down-configuration end point coordinate [x;y]
            s1 = sind(fiveLinkage.theta(1));
            c1 = cosd(fiveLinkage.theta(1));
            s2 = sind(fiveLinkage.theta(2));
            c2 = cosd(fiveLinkage.theta(2));
            
            denominator =  (2 * fiveLinkage.r(3) + fiveLinkage.r(1) * c2 - fiveLinkage.r(1) * c1);
            if (abs(denominator) < 1E-6)
                %infinitely many solutions
                nSol = -1;
                up = [];
                down = [];
                return;
            end
            
            e = fiveLinkage.r(1) * (s1 - s2)/ denominator;
            f = fiveLinkage.r(1) * fiveLinkage.r(3) * (c2 + c1) / denominator;
            d = 1 + e^2;
            g = 2 * (e * f - e * fiveLinkage.r(1)*c1 + e * fiveLinkage.r(3) - fiveLinkage.r(1) * s1);
            h = f^2 - 2*f*(fiveLinkage.r(1)*c1 - fiveLinkage.r(3)) - 2 * fiveLinkage.r(1) * fiveLinkage.r(3) * c1 + fiveLinkage.r(3)^2 + fiveLinkage.r(1)^2 - fiveLinkage.r(2)^2;
            
            det = g^2 - 4*d*h;
            
            if (det < -1E-6)
                %no solution
                nSol = 0;
                up = [];
                down = [];
                return;
            end
            
            if (det < 1E-6)
                y = -g / 2 / d;
                x = e * y + f;
                nSol = 1;
                up = [x; y];
                down = [x; y];
                return;
            end
            
            y1 = (-g + sqrt(det)) / 2 /d;
            x1 = e * y1 + f;
            
            y2 = (-g - sqrt(det)) / 2 /d;
            x2 = e * y2 + f;
            
            nSol = 2;
            up = [x1; y1];
            down = [x2; y2];
        end
        
        function [pp, pn, np, nn] = inverseKinematics(fiveLinkage, p)
            %inverse kinematics
            %input arguments:
            %               fiveLinkage: RRRRR object
            %               p: end point coordinate
            %return value:
            %               pp: '++' mode [theta1; theat2]
            %               pn: '+-' mode [theta1; theat2]
            %               np: '-+' mode [theta1; theat2]
            %               nn: '--' mode [theta1; theat2]
            r1 = fiveLinkage.r(1);
            r2 = fiveLinkage.r(2);
            r3 = fiveLinkage.r(3);
            x = p(1);
            y = p(2);
            
            a1 = r1^2 + y^2 + (x + r3)^2 - r2^2 + 2 * (x + r3) * r1;
            b1 = -4 * y * r1;
            c1 = r1^2 + y^2 + (x + r3)^2 - r2^2 - 2 *(x + r3) * r1;
            a2 = r1^2 + y^2 + (x - r3)^2 - r2^2 + 2 * (x - r3) * r1;
            b2 = b1;
            c2 = r1^2 + y^2 + (x - r3)^2 - r2^2 - 2 *(x - r3) * r1;
            
            det1 = b1^2 - 4 * a1 * c1;
            det2 = b2^2 - 4 * a2 * c2;
            if ( det1 < -1E-6 || det2 < -1E-6)
                pp = [];
                pn = [];
                np = [];
                nn = [];
                return;
            end
            
            if (det1 < 1E-6)
                det1 = 0;
            end
            
            if (det2 < 1E-6)
                det2 = 0;
            end
            
            pp = [0; 0];
            pn = [0; 0];
            np = [0; 0];
            nn = [0; 0];
            
            %sigma1 = 1
            Y = -b1 + sqrt(det1);
            X = 2 * a1;
            theta1 = 2 * atan2d(Y, X);
            pp(1) = theta1;
            pn(1) = theta1;
            
            %sigma1 = -1
            Y = -b1 - sqrt(det1);
            X = 2 * a1;
            theta1 = 2 * atan2d(Y, X);
            np(1) = theta1;
            nn(1) = theta1;
            
            %sigma2 = 1
            Y = -b2 + sqrt(det2);
            X = 2 * a2;
            theta2 = 2 * atan2d(Y, X);
            pp(2) = theta2;
            np(2) = theta2;
            
            %sigma2 = -1
            Y = -b2 - sqrt(det2);
            X = 2 * a2;
            theta2 = 2 * atan2d(Y, X);
            pn(2) = theta2;
            nn(2) = theta2;
            
            return;
        end
        
    end
end
