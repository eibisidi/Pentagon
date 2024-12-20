classdef RRRRR
    %basic 5 bar linkage class
    properties( Constant = true )
        CONF_NONE = -1
        CONF_UP   = 1
        CONF_DOWN = 2
        MODE_NONE = -1
        MODE_PP   = 1
        MODE_PN   = 2
        MODE_NP   = 3
        MODE_NN   = 4
    end
    
    properties (Access = public)
        %Fixed values
        r % linkage length [r1;r2;r3]
        A1 %A1 coordinate [-r3; 0]
        A2 %A2 coordinate [r3; 0]
        
        %Current values
        theta %active joint degree with x axis [theta1;theta2]
        B1 %B1 coordinate dependent on theta1
        B2 %B2 coordinate dependent on theta2
        passive_theta %active joint degree with x axis [theta3;theta4]
        P  %end point coordinate[x; y]
        current_configuration;     %-1 no solution; 0 down; 1 up
        initial_configuration;     % 0 down; 1 up
        
        %FK results
        fk_B1;
        fk_B2;
        fk_nSol;  %forward kinematics solution number; -1 Inf, 0 no, 1/2
        fk_P;     %forward  [up down]
        fk_modes; %work mode [up down]
        fk_passive_theta; %passive joint thetas [up down]
        
        %IF results
        ik_nSol;    %0/4
        ik_theta;   %active joint theta [pp pn np nn]
        ik_B1;
        ik_B2;
        ik_passive_theta;  %passvie joint theta [pp pn np nn]
        ik_configuration;  %configuration [pp pn np nn]
        %         fk_up;    %up-configuration coordinate[x;y]
        %         fk_down;  %down-configuration coordinate[x;y]
        %         fk_up_theta; %up-configuration PB1&PB2 degree with axis x[theta3; theta4]
        %         fk_down_theta; %down-configuration PB1&PB2 degree with axis x[theta3; theta4]
        %         current_configuration;     %-1 no solution; 0 down; 1 up
        %         initial_configuration;     % 0 down; 1 up
        %         current_position;          % end-point coordinate [x; y]
        %         ik_pp; % ++mode [theta1; theta2]
        %         ik_pn; % +-mode [theta1; theta2]
        %         ik_np; % -+mode [theta1; theta2]
        %         ik_nn; % --mode [theta1; theta2]
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
        
        function mode = string2mode(fiveLinkage, string)
            switch string
                case '++'
                    mode = fiveLinkage.MODE_PP;
                case '+-'
                    mode = fiveLinkage.MODE_PN;
                case '-+'
                    mode = fiveLinkage.MODE_NP;
                case '--'
                    mode = fiveLinkage.MODE_NN;
                otherwise
                    mode = fiveLinkage.MODE_NONE;
            end
        end
        
        function modeStr = getWorkModeString(fiveLinkage, configuration)
            mode = fiveLinkage.getWorkMode(configuration);
            switch mode
                case fiveLinkage.MODE_PP
                    modeStr = '++';
                case fiveLinkage.MODE_PN
                    modeStr = '+-';
                case fiveLinkage.MODE_NP
                    modeStr = '-+';
                case fiveLinkage.MODE_NN
                    modeStr = '--';
                otherwise
                    modeStr = 'xx';
            end
            return;
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
                mode = fiveLinkage.MODE_NONE;
                return;
            end

            p =  fiveLinkage.fk_P(:, configuration);
            mode = fiveLinkage.calcWorkMode(fiveLinkage.fk_B1, fiveLinkage.fk_B2, p);
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
        
        function mode = calcWorkMode(fiveLinkage, b1, b2, p)
            A1C1 = p - fiveLinkage.A1;      %vector A1P
            A1B1 = b1 - fiveLinkage.A1;     %vector A1B1
            leftCross = A1C1(1) * A1B1(2) - A1B1(1) * A1C1(2);
            
            A2C1 = p  - fiveLinkage.A2;
            A2B2 = b2 - fiveLinkage.A2;
            rightCross = A2C1(1) * A2B2(2) - A2B2(1) * A2C1(2);
            
            if (leftCross >= -1E-6  && rightCross >= -1E-6)
                mode = fiveLinkage.MODE_PP;
            elseif (leftCross >= -1E-6 && rightCross < -1E-6)
                mode = fiveLinkage.MODE_PN;
            elseif (leftCross < -1E-6 && rightCross >= -1E-6)
                mode = fiveLinkage.MODE_NP;
            else
                mode = fiveLinkage.MODE_NN;
            end
            return;
        end
        
        function [b1, b2] = calcB1B2(fiveLinkage, joint_theta)
            s1 = sind(joint_theta(1));
            c1 = cosd(joint_theta(1));
            s2 = sind(joint_theta(2));
            c2 = cosd(joint_theta(2));
            b1 = [fiveLinkage.r(1) * c1 - fiveLinkage.r(3); fiveLinkage.r(1) * s1];
            b2 = [fiveLinkage.r(1) * c2 + fiveLinkage.r(3); fiveLinkage.r(1) * s2];
        end
        
        function passive_theta = calcPassiveTheta(~, b1, b2, p)
            u = p - b1;
            v = p - b2;
            passive_theta = [atan2d(u(2), u(1)); atan2d(v(2), v(1))];
        end
        
        function configuration = calcConfiguration(fiveLinkage, b1, b2, p)
            midY = (b1(2) + b2(2)) / 2;
            
            if (p(2) >= midY)
                configuration = fiveLinkage.CONF_UP;%up
            else
                configuration = fiveLinkage.CONF_DOWN; %down
            end
        end
        
        function [fiveLinkage] = forwardKinematics(fiveLinkage, newTheta)
            %forward kinematics
            %input arguments:
            %               r: [r1; r2; r3] column vector
            %               theta: [theta1; theta2] column vector
            %return value:
            %           b1: B1 point coordinate
            %           b2: B2 point coordinate
            %           nSol: number of solutions.
            %           	-1: infinitely many solutions, B1 and B2 coincides
            %               0: no solution 1/2
            %               1: up and down is the same point
            %               2: up and down is different
            %           up: up-configuration end point coordinate [x;y]
            %           down: down-configuration end point coordinate [x;y]
            %           up_theta: up-configuration [theta3; theta4]
            %           down_theta: down-configuration [theta3; theta4]
            s1 = sind(newTheta(1));
            c1 = cosd(newTheta(1));
            s2 = sind(newTheta(2));
            c2 = cosd(newTheta(2));
            
            b1 = [fiveLinkage.r(1) * c1 - fiveLinkage.r(3); fiveLinkage.r(1) * s1];
            b2 = [fiveLinkage.r(1) * c2 + fiveLinkage.r(3); fiveLinkage.r(1) * s2];
            fiveLinkage.fk_B1 = b1;
            fiveLinkage.fk_B2 = b2;
            
            denominator =  (2 * fiveLinkage.r(3) + fiveLinkage.r(1) * c2 - fiveLinkage.r(1) * c1);
            if (abs(denominator) < 1E-6)
                %infinitely many solutions
                fiveLinkage.fk_nSol = -1;
                fiveLinkage.fk_P = [];
                fiveLinkage.fk_modes = [];
                fiveLinkage.fk_passive_theta = [];
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
                fiveLinkage.fk_nSol = 0;
                fiveLinkage.fk_P = [];
                fiveLinkage.fk_modes = [];
                fiveLinkage.fk_passive_theta = [];
                return;
            end
            
            if (det < 1E-6)
                y = -g / 2 / d;
                x = e * y + f;
                up = [x; y];
                u = up - b1;
                v = up - b2;
                up_mode = calcWorkMode(fiveLinkage, b1, b2, up);
                up_theta = [atan2d(u(2), u(1)); atan2d(v(2), v(1))];
                
                fiveLinkage.fk_nSol = 1;
                fiveLinkage.fk_P = [up up];
                fiveLinkage.fk_modes = [up_mode up_mode];
                fiveLinkage.fk_passive_theta = [up_theta up_theta];
                
                return;
            end
            
            y1 = (-g + sqrt(det)) / 2 /d;
            x1 = e * y1 + f;
            
            y2 = (-g - sqrt(det)) / 2 /d;
            x2 = e * y2 + f;
            
            up = [x1; y1];
            down = [x2; y2];
            up_mode = calcWorkMode(fiveLinkage, b1, b2, up);
            down_mode = calcWorkMode(fiveLinkage, b1, b2, down);
            u = up - b1;
            v = up - b2;
            up_theta = [atan2d(u(2), u(1)); atan2d(v(2), v(1))];
            u = down - b1;
            v = down - b2;
            down_theta = [atan2d(u(2), u(1)); atan2d(v(2), v(1))];
            
            fiveLinkage.fk_nSol = 2;
            fiveLinkage.fk_P = [up down];
            fiveLinkage.fk_modes = [up_mode down_mode];
            fiveLinkage.fk_passive_theta = [up_theta down_theta];
        end
        
        function [fiveLinkage] = inverseKinematics(fiveLinkage, newP)
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
            x = newP(1);
            y = newP(2);
            
            a1 = r1^2 + y^2 + (x + r3)^2 - r2^2 + 2 * (x + r3) * r1;
            b1 = -4 * y * r1;
            c1 = r1^2 + y^2 + (x + r3)^2 - r2^2 - 2 *(x + r3) * r1;
            a2 = r1^2 + y^2 + (x - r3)^2 - r2^2 + 2 * (x - r3) * r1;
            b2 = b1;
            c2 = r1^2 + y^2 + (x - r3)^2 - r2^2 - 2 *(x - r3) * r1;
            
            det1 = b1^2 - 4 * a1 * c1;
            det2 = b2^2 - 4 * a2 * c2;
            if ( det1 < -1E-6 || det2 < -1E-6)
                fiveLinkage.ik_nSol = 0;
                fiveLinkage.ik_theta = [];
                fiveLinkage.ik_B1 = [];
                fiveLinkage.ik_B2 = [];
                fiveLinkage.ik_passive_theta = [];
                fiveLinkage.ik_configuration = [];
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
            
            [pp_b1, pp_b2] = calcB1B2(fiveLinkage, pp);
            [pn_b1, pn_b2] = calcB1B2(fiveLinkage, pn);
            [np_b1, np_b2] = calcB1B2(fiveLinkage, np);
            [nn_b1, nn_b2] = calcB1B2(fiveLinkage, nn);
            
            pp_passive_theta = calcPassiveTheta(fiveLinkage, pp_b1, pp_b2, newP);
            pn_passive_theta = calcPassiveTheta(fiveLinkage, pn_b1, pn_b2, newP);
            np_passive_theta = calcPassiveTheta(fiveLinkage, np_b1, np_b2, newP);
            nn_passive_theta = calcPassiveTheta(fiveLinkage, nn_b1, nn_b2, newP);
            
            pp_conf = calcConfiguration(fiveLinkage, pp_b1, pp_b2, newP);
            pn_conf = calcConfiguration(fiveLinkage, pn_b1, pn_b2, newP);
            np_conf = calcConfiguration(fiveLinkage, np_b1, np_b2, newP);
            nn_conf = calcConfiguration(fiveLinkage, nn_b1, nn_b2, newP);
            
            fiveLinkage.ik_nSol = 4;
            fiveLinkage.ik_theta = [pp pn np nn];
            fiveLinkage.ik_B1 = [pp_b1 pn_b1 np_b1 nn_b1];
            fiveLinkage.ik_B2 = [pp_b2 pn_b2 np_b2 nn_b2];
            fiveLinkage.ik_passive_theta = [pp_passive_theta pn_passive_theta np_passive_theta nn_passive_theta];
            fiveLinkage.ik_configuration = [pp_conf pn_conf np_conf nn_conf];
            return;
        end
        
        function fiveLinkage = setTheta(fiveLinkage, newTheta)
            fiveLinkage = fiveLinkage.forwardKinematics(newTheta);
            fiveLinkage.theta = newTheta;
            fiveLinkage.B1 = fiveLinkage.fk_B1;
            fiveLinkage.B2 = fiveLinkage.fk_B2;
            
            if (fiveLinkage.fk_nSol < 1)
                %no solution / INF solutions
                fiveLinkage.passive_theta = [];
                fiveLinkage.P = [];
                fiveLinkage.current_configuration = fiveLinkage.CONF_NONE;
                return;
            end
            
            %there is solutions
            if (fiveLinkage.current_configuration < 0)
                fiveLinkage.current_configuration = fiveLinkage.initial_configuration;
            else
                diff_up   = fiveLinkage.fk_P(:,fiveLinkage.CONF_UP) - fiveLinkage.P;
                diff_down = fiveLinkage.fk_P(:,fiveLinkage.CONF_DOWN) - fiveLinkage.P;
                if (norm(diff_up) < norm(diff_down))
                    fiveLinkage.current_configuration = fiveLinkage.CONF_UP;
                else
                    fiveLinkage.current_configuration = fiveLinkage.CONF_DOWN;
                end
            end
            
            fiveLinkage.P = fiveLinkage.fk_P(:,fiveLinkage.current_configuration);
            fiveLinkage.passive_theta = fiveLinkage.fk_passive_theta(:,fiveLinkage.current_configuration);
        end
        
        function valid = isValid(fiveLinkage)
            valid = 1;
            if (fiveLinkage.current_configuration == fiveLinkage.CONF_NONE)
                valid = 0;
            end
            return;
        end
        
        function fiveLinkage = setP(fiveLinkage, newP, modeString)
            fiveLinkage = fiveLinkage.inverseKinematics(newP);
            if (fiveLinkage.ik_nSol < 4)
                return;
            end
            
            %IK success, update current status
            mode = fiveLinkage.string2mode(modeString);
            
            fiveLinkage.theta = fiveLinkage.ik_theta(:, mode);
            fiveLinkage.B1 = fiveLinkage.ik_B1(:, mode);
            fiveLinkage.B2 = fiveLinkage.ik_B2(:, mode);
            
            fiveLinkage.passive_theta = fiveLinkage.ik_passive_theta(:, mode);
            fiveLinkage.P = newP;
            fiveLinkage.current_configuration = fiveLinkage.ik_configuration(:, mode);
            
            return;
        end
    end
end
