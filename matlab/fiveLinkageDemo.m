clear;
close all;

global fiveLinkage;
fiveLinkage = RRRRR;
fiveLinkage.r = [1.2; 1.0; 0.8];
%fiveLinkage.r = [1; 1; 0.4];  %lenth parameters of edcational robots
fiveLinkage.theta = [0; 0];
fiveLinkage.A1 = [-fiveLinkage.r(3); 0];
fiveLinkage.A2 = [fiveLinkage.r(3); 0];
fiveLinkage.current_configuration = fiveLinkage.CONF_NONE;
fiveLinkage.initial_configuration = fiveLinkage.CONF_UP;  %use up configuration when startup

%Rotation axis
global L phi tooltip thetaR;
L = 0.2;    %link length
phi = 0;    %direction angle wrt. x axis
tooltip = []; %tooltip coordinate [x;y]
thetaR = 0;      %R joint roation angle

global steps;
steps = 0;

global h_a1b1 h_a2b2 h_b1c1b2 h_b1c2b2 h_end_effector;
h_a1b1 = []; %matlab plot handle of active link A1B1
h_a2b2 = []; %matlab plot handle of active link A2B2
h_b1c1b2 = []; %matlab plot handle of up-configuration
h_b1c2b2 = []; %matlab plot handle of down-configuration
h_end_effector = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%UI creation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
UIFigure = uifigure;
UIFigure.Position = [100 100 310 600];
UIFigure.Name = 'UI Figure';

UIAxesFigure = figure;
UIAxesFigure.Position = [420 100 600 500];
global UIAxes;
UIAxes = axes(UIAxesFigure);
title(UIAxes, '5 linkage');
xlabel(UIAxes, 'X');
ylabel(UIAxes, 'Y');
set(UIAxes, 'ButtonDownFcn', @AxesMouseClicked);

% Create theta1SliderLabel
theta1SliderLabel = uilabel(UIFigure);
theta1SliderLabel.HorizontalAlignment = 'right';
theta1SliderLabel.Position = [22,579,39,22];
theta1SliderLabel.Text = 'theta1';

global theta1Slider theta2Slider;
% Create theta1Slider
theta1Slider = uislider(UIFigure);
theta1Slider.Limits = [-360 360];
theta1Slider.Orientation = 'vertical';
theta1Slider.ValueChangingFcn = @theta1SliderValueChanged;
theta1Slider.ValueChangingFcn = @theta1SliderValueChanged;
theta1Slider.Position = [22,9,3,571];
theta1Slider.Value = fiveLinkage.theta(1);

% Create theta2SliderLabel
theta2SliderLabel = uilabel(UIFigure);
theta2SliderLabel.HorizontalAlignment = 'right';
theta2SliderLabel.Position = [99,579,39,22];
theta2SliderLabel.Text = 'theta2';

% Create theta2Slider
theta2Slider = uislider(UIFigure);
theta2Slider.Limits = [-360 360];
theta2Slider.Orientation = 'vertical';
theta2Slider.ValueChangingFcn = @theta2SliderValueChanged;
theta2Slider.ValueChangingFcn = @theta2SliderValueChanged;
theta2Slider.Position = [97,9,3,571];
theta1Slider.Value = fiveLinkage.theta(2);

% Create ForwardKinematicsLabel
ForwardKinematicsLabel = uilabel(UIFigure);
ForwardKinematicsLabel.Position =  [172 579 115 22];
ForwardKinematicsLabel.Text = 'Forward Kinematics:';

global UpLabel upModeLabel DownLabel downModeLabel;
% Create UpLabel
UpLabel = uilabel(UIFigure);
UpLabel.Position = [172 553 25 22];
UpLabel.Text = 'Up:';

% Create upModeLabel
upModeLabel = uilabel(UIFigure);
upModeLabel.FontName = 'Courier';
upModeLabel.FontSize = 16;
upModeLabel.Position = [209 553 63 22];
upModeLabel.Text = 'upMode';

% Create DownLabel
DownLabel = uilabel(UIFigure);
DownLabel.Position = [172 532 40 22];
DownLabel.Text = 'Down:';

% Create downModeLabel
downModeLabel = uilabel(UIFigure);
downModeLabel.FontName = 'Courier';
downModeLabel.FontSize = 16;
downModeLabel.Position = [211 532 82 22];
downModeLabel.Text = 'downMode';

% Create ModeKnobLabel
UIModeKnobLabel = uilabel(UIFigure);
UIModeKnobLabel.HorizontalAlignment = 'center';
UIModeKnobLabel.Position = [199.5 407 63 22];
UIModeKnobLabel.Text = 'WorkMode';

% Create ModeKnob
global UIModeKnob;
UIModeKnob = uiknob(UIFigure, 'discrete');
UIModeKnob.Items = {'++', '+-', '-+', '--'};
UIModeKnob.Position = [200 444 60 60];
UIModeKnob.Value = '+-';
UIModeKnob.ValueChangedFcn = @KnobValueChanged;

% Create phiSliderLabel
phiSliderLabel = uilabel(UIFigure);
phiSliderLabel.HorizontalAlignment = 'right';
phiSliderLabel.Position = [212 357 25 22];
phiSliderLabel.Text = 'phi';

% Create phiSlider
phiSlider = uislider(UIFigure);
phiSlider.Limits = [-180 180];
phiSlider.Orientation = 'vertical';
phiSlider.Position = [212 142 3 216];
phiSlider.Value = phi;
phiSlider.ValueChangingFcn = @phiSliderValueChanged;
phiSlider.ValueChangingFcn = @phiSliderValueChanged;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%draw singularity curve
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[fp_C1o, fp_C1i, fp_C2o, fp_C2i, fp_CCoin_U, fp_CCoin_D, fp_CCol] = loci(UIAxes, fiveLinkage.r);
fp_C1o.PickableParts = 'none';
fp_C1i.PickableParts = 'none';
fp_C2o.PickableParts = 'none';
fp_C2i.PickableParts = 'none';
fp_CCoin_U.PickableParts = 'none';
fp_CCoin_D.PickableParts = 'none';
fp_CCol.PickableParts = 'none';
handleThetaChanged(theta1Slider.Value, theta2Slider.Value, thetaR);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%UI Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta1SliderValueChanged(~, event)
    global theta2Slider;
    global thetaR;
    handleThetaChanged(event.Value, theta2Slider.Value, thetaR);
end

function theta2SliderValueChanged(~, event)
    global theta1Slider;
    global thetaR;
    handleThetaChanged(theta1Slider.Value, event.Value, thetaR);
end

function phiSliderValueChanged(~, event)
    global UIModeKnob;
    global phi tooltip;
    phi = event.Value;
    handleEndPointChanged(tooltip, UIModeKnob.Value);
end

function  AxesMouseClicked(Object, ~)
    global UIModeKnob;
    global steps;
    cp = Object.CurrentPoint(1,:);
    endEffector = [cp(1) ; cp(2)];
    if (steps == 0)
        handleEndPointChanged(endEffector, UIModeKnob.Value);
        return;
    end
    
    global fiveLinkage;
    global L phi R;
    startJoint = [fiveLinkage.theta(1); fiveLinkage.theta(2); R];
    
    p = endEffector;
    if ( L > 0)
        p(1) = endEffector(1) - L * cosd(phi);
        p(2) = endEffector(2) - L * sind(phi);
    end
    [fiveLinkage.ik_pp, fiveLinkage.ik_pn, fiveLinkage.ik_np, fiveLinkage.ik_nn] = fiveLinkage.inverseKinematics(p);
    %disp([fiveLinkage.ik_pp, fiveLinkage.ik_pn, fiveLinkage.ik_np, fiveLinkage.ik_nn] );

    switch  UIModeKnob.Value
    case '++'
        solution = fiveLinkage.ik_pp;
    case '+-'
        solution = fiveLinkage.ik_pn;
    case '-+'
        solution = fiveLinkage.ik_np;
    otherwise
        solution = fiveLinkage.ik_nn;
    end
    
    if (isempty(solution)) %IK no solution
        return;
    end
    
    fiveLinkage.current_position = p;
    fiveLinkage.current_configuration = fiveLinkage.getConfiguration( UIModeKnob.Value);
	fiveLinkage.theta = solution;
    [fiveLinkage.B1,fiveLinkage.B2, fiveLinkage.fk_nSol, fiveLinkage.fk_up, fiveLinkage.fk_down, fiveLinkage.fk_up_theta, fiveLinkage.fk_down_theta] = fiveLinkage.forwardKinematics();
    
    if (fiveLinkage.current_configuration == 1)
        rotation = phi - fiveLinkage.fk_up_theta(1);
    else
        rotation = phi - fiveLinkage.fk_down_theta(1);
    end
    endJoint = [solution(1);solution(2); rotation];
    
    distance = endJoint - startJoint;
    for i = 1:1:steps
        disp(i);
        ratio = i / steps;
        t1 = startJoint(1) + ratio * distance(1);
        t2 = startJoint(2) + ratio * distance(2);
        fiveLinkage.theta = [t1; t2];
        handleThetaChanged();
        pause(0.1);
    end
end

function KnobValueChanged(~, event)
    global tooltip;
    handleEndPointChanged(tooltip, event.Value);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handleEndPointChanged(newToolTip, mode)
    %Input argument
    %   tooltip: end-effector coordinate [x; y]
    %   mode: work mode requireds
    global fiveLinkage;
    global theta1Slider theta2Slider;
    global L phi thetaR;
    p = newToolTip;
    if ( L > 0)
        p(1) = newToolTip(1) - L * cosd(phi);
        p(2) = newToolTip(2) - L * sind(phi);
    end
    
    fiveLinkage = fiveLinkage.setP(p, mode);
    
    theta1Slider.Value = fiveLinkage.theta(1);
    theta2Slider.Value = fiveLinkage.theta(2);
    handleThetaChanged(fiveLinkage.theta(1), fiveLinkage.theta(2), thetaR);
end

function handleThetaChanged(theta1, theta2, thetaR)
    global fiveLinkage;
    fiveLinkage = fiveLinkage.setTheta([theta1; theta2]);
    
    global L phi tooltip;
    if (fiveLinkage.isValid())
        %update tooltip coordinate
        
        endX = fiveLinkage.P(1) + L * cosd(phi);
        endY = fiveLinkage.P(2) + L * sind(phi);
        tooltip = [endX; endY];
    end
    
    updateUI();
end

function updateUI()
    drawActiveLink();
    drawPassiveLink();
    drawEndEffector();
    updateFkModeLabel();
end

function results = drawActiveLink()
    global h_a1b1 h_a2b2;
    global fiveLinkage;
    global UIAxes;
    if (~isempty(h_a1b1))
        delete(h_a1b1);
    end
    if (~isempty(h_a2b2))
        delete(h_a2b2);
    end
    h_a1b1 = plot(UIAxes, [fiveLinkage.A1(1) fiveLinkage.B1(1)] , [fiveLinkage.A1(2) fiveLinkage.B1(2)], '-go');
    h_a1b1.PickableParts = 'none';
    h_a2b2 = plot(UIAxes, [fiveLinkage.A2(1) fiveLinkage.B2(1)] , [fiveLinkage.A2(2) fiveLinkage.B2(2)], '-go');
    h_a2b2.PickableParts = 'none';
    results = 0;
end
        
function drawPassiveLink()
    global h_b1c1b2 h_b1c2b2;
    global fiveLinkage;
    global UIAxes;
    if (~isempty(h_b1c1b2))
        delete(h_b1c1b2);
    end
    
    if (~isempty(h_b1c1b2))    
        delete(h_b1c2b2);
    end
    
    if (~fiveLinkage.isValid())
        return;
    end

    upLineSpec  = '-mo';
    downLineSpec = ':mo';
    
    if (fiveLinkage.current_configuration == fiveLinkage.CONF_DOWN)
        upLineSpec  = ':mo';
        downLineSpec = '-mo';
    end

    upP = fiveLinkage.fk_P(:, fiveLinkage.CONF_UP);
    
    h_b1c1b2 = plot(UIAxes, [fiveLinkage.B1(1) upP(1) fiveLinkage.B2(1)] , [fiveLinkage.B1(2) upP(2) fiveLinkage.B2(2)], upLineSpec);
    h_b1c1b2.PickableParts = 'none';
    
    if (fiveLinkage.fk_nSol == 2)
        downP = fiveLinkage.fk_P(:, fiveLinkage.CONF_DOWN);
        h_b1c2b2 = plot(UIAxes, [fiveLinkage.B1(1) downP(1) fiveLinkage.B2(1)] , [fiveLinkage.B1(2) downP(2) fiveLinkage.B2(2)], downLineSpec);
        h_b1c2b2.PickableParts = 'none';
    end                   
end

function drawEndEffector()
    global L tooltip;
    global fiveLinkage;
    global h_end_effector;
    global UIAxes;
    
    if (L <= 0)
        return;
    end
    
    if (~isempty(h_end_effector))    
        delete(h_end_effector);
    end
    
    if (~fiveLinkage.isValid())
        return;
    end
    
    h_end_effector = plot(UIAxes, [fiveLinkage.P(1) tooltip(1)] , [fiveLinkage.P(2) tooltip(2)], '-cx');
    h_end_effector.PickableParts = 'none';
end

function  updateFkModeLabel()
    global UpLabel upModeLabel DownLabel downModeLabel UIModeKnob;
    global fiveLinkage;

    upModeLabel.Text = fiveLinkage.getWorkModeString(fiveLinkage.CONF_UP);
    downModeLabel.Text = fiveLinkage.getWorkModeString(fiveLinkage.CONF_DOWN);

    if (fiveLinkage.current_configuration == fiveLinkage.CONF_UP)
        UpLabel.BackgroundColor = [0.00,1.00,0.00];
        DownLabel.BackgroundColor = 'none';
        UIModeKnob.Value = upModeLabel.Text;
    elseif (fiveLinkage.current_configuration == fiveLinkage.CONF_DOWN)
        UpLabel.BackgroundColor = 'none';
        DownLabel.BackgroundColor = [0.00,1.00,0.00];
        UIModeKnob.Value = downModeLabel.Text;
    else
        UpLabel.BackgroundColor = 'none';
        DownLabel.BackgroundColor = 'none';
    end               
end