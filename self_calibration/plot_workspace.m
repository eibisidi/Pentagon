% Five-bar parallel robot parameters
proximal_left  = 270;   % mm
distal_left    = 370;   % mm
proximal_right = 270;   % mm
distal_right   = 370;   % mm
base_distance  = 200;   % mm

step = 5; % degrees

% --- Full mechanical workspace ---
full_positions = [];
for t1 = 0 : step : 360
    for t2 = 0 : step : 360
        pos = forward_5bar(t1, t2, ...
            proximal_left, distal_left, proximal_right, distal_right, base_distance);
        if ~isempty(pos)
            full_positions(end+1,:) = pos; %#ok<AGROW>
        end
    end
end

% --- Limited joint range workspace ---
theta1_min = -50;
theta1_max = 110;
theta2_min = 60;
theta2_max = 220;
limit_positions = [];
joint_pairs = [];
for t1 = theta1_min : step : theta1_max
    for t2 = theta2_min : step : theta2_max
        pos = forward_5bar(t1, t2, ...
            proximal_left, distal_left, proximal_right, distal_right, base_distance);
        if ~isempty(pos)
            if (pos(2) > 100 && pos(2) < 500)
            limit_positions(end+1,:) = pos; %#ok<AGROW>
            joint_pairs = [joint_pairs, [t2; t1]];
            end
        end
    end
end

csvwrite('theta_pairs.csv', joint_pairs);

% --- Plot ---
figure;
scatter(full_positions(:,1), full_positions(:,2), 5, [0.7 0.7 0.7], 'filled'); % gray
hold on;
scatter(limit_positions(:,1), limit_positions(:,2), 5, 'b', 'filled');
xlabel('X [mm]');
ylabel('Y [mm]');
axis equal;
grid on;
title('Five-Bar Parallel Robot Workspace');
legend('Full mechanical limits', 'Given joint limits');

% --- Forward kinematics function ---
function pos = forward_5bar(t1_deg, t2_deg, L1, L2, L3, L4, base_dist)
    t1 = deg2rad(t1_deg);
    t2 = deg2rad(t2_deg);
    
    left_base  = [0, 0];
    right_base = [base_dist, 0];
    
    left_joint  = left_base  + L1 * [cos(t1), sin(t1)];
    right_joint = right_base + L3 * [cos(t2), sin(t2)];
    
    d = norm(right_joint - left_joint);
    if d > L2 + L4 || d < abs(L2 - L4)
        pos = [];
        return;
    end
    
    a = (L2^2 - L4^2 + d^2) / (2*d);
    h = sqrt(max(L2^2 - a^2, 0));
    
    P2 = left_joint + a * (right_joint - left_joint) / d;
    offset = h * [-(right_joint(2) - left_joint(2)) / d, ...
                   (right_joint(1) - left_joint(1)) / d];
    
    pos = P2 + offset; % choose upper intersection
end
