clear;
close all;

global fiveLinkage;
fiveLinkage = RRRRR;
fiveLinkage.r = [1.2; 1.0; 0.8];
%fiveLinkage.r = [1; 1; 0.4];  %lenth parameters of edcational robots
fiveLinkage.theta = [0; 0];
fiveLinkage.A1 = [-fiveLinkage.r(3); 0];
fiveLinkage.A2 = [fiveLinkage.r(3); 0];
fiveLinkage.current_configuration = -1;
fiveLinkage.initial_configuration = 1;  %use up configuration when startup

global h_a1b1 h_a2b2 h_b1c1b2 h_b1c2b2;
h_a1b1 = []; %matlab plot handle of active link A1B1
h_a2b2 = []; %matlab plot handle of active link A2B2
h_b1c1b2 = []; %matlab plot handle of up-configuration
h_b1c2b2 = []; %matlab plot handle of down-configuration

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%UI creation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
UIFigure = uifigure;
UIFigure.Position = [100 100 310 600];
UIFigure.Name = 'UI Figure';

UIAxesFigure = figure;
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
UIModeKnobLabel.Position = [190 390 60 22];
UIModeKnobLabel.Text = 'WorkMode';

% Create ModeKnob
global UIModeKnob;
UIModeKnob = uiknob(UIFigure, 'discrete');
UIModeKnob.Items = {'++', '+-', '-+', '--'};
UIModeKnob.Position = [193 416 60 60];
UIModeKnob.Value = '+-';
UIModeKnob.ValueChangedFcn = @KnobValueChanged;

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
handleThetaChanged();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%UI Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta1SliderValueChanged(~, event)
    global fiveLinkage;
    fiveLinkage.theta(1) = event.Value;
    handleThetaChanged();
end

function theta2SliderValueChanged(~, event)
    global fiveLinkage;
    fiveLinkage.theta(2) = event.Value;
    handleThetaChanged();
end

function  AxesMouseClicked(Object, ~)
    global UIModeKnob;
    
    cp = Object.CurrentPoint(1,:);
    p = [cp(1) ; cp(2)];
    handleEndPointChanged(p, UIModeKnob.Value);
end

function KnobValueChanged(~, event)
    global fiveLinkage;
    if (isempty(fiveLinkage.current_position))
        return;
    end
    handleEndPointChanged(fiveLinkage.current_position, event.Value);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handleEndPointChanged(p, mode)
    global fiveLinkage;
    global theta1Slider theta2Slider;
    [fiveLinkage.ik_pp, fiveLinkage.ik_pn, fiveLinkage.ik_np, fiveLinkage.ik_nn] = fiveLinkage.inverseKinematics(p);
    %disp([fiveLinkage.ik_pp, fiveLinkage.ik_pn, fiveLinkage.ik_np, fiveLinkage.ik_nn] );

    switch mode
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
    
    theta1Slider.Value = solution(1);
    theta2Slider.Value = solution(2);
    fiveLinkage.current_configuration = -1;
    fiveLinkage.current_position = p;
    fiveLinkage.initial_configuration = fiveLinkage.getConfiguration(mode);
    fiveLinkage.theta = solution;
    handleThetaChanged();
end

function results = handleThetaChanged()
    global fiveLinkage;
    fiveLinkage.B1 = [fiveLinkage.r(1) * cosd(fiveLinkage.theta(1)) - fiveLinkage.r(3); fiveLinkage.r(1) * sind(fiveLinkage.theta(1))];
    fiveLinkage.B2 = [fiveLinkage.r(1) * cosd(fiveLinkage.theta(2)) + fiveLinkage.r(3); fiveLinkage.r(1) * sind(fiveLinkage.theta(2))];
    drawActiveLink();

    %do forward kinematics
    [fiveLinkage.fk_nSol, fiveLinkage.fk_up, fiveLinkage.fk_down] = fiveLinkage.forwardKinematics();

    if (fiveLinkage.fk_nSol < 1)
        %no solution
        fiveLinkage.current_configuration = -1;
        fiveLinkage.current_position = [];
    elseif (fiveLinkage.current_configuration < 0)
        fiveLinkage.current_configuration = fiveLinkage.initial_configuration;
    else
        diff_up   = fiveLinkage.fk_up - fiveLinkage.current_position;
        diff_down = fiveLinkage.fk_down - fiveLinkage.current_position;
        if (norm(diff_up) < norm(diff_down))
            fiveLinkage.current_configuration = 1;
        else
            fiveLinkage.current_configuration = 0;
        end
    end

    if (fiveLinkage.current_configuration == 1)
        fiveLinkage.current_position = fiveLinkage.fk_up;
    else
        fiveLinkage.current_position = fiveLinkage.fk_down;
    end

    drawPassiveLink();

    updateFkModeLabel();
    results = 0;
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
        
function results = drawPassiveLink()
    global h_b1c1b2 h_b1c2b2;
    global fiveLinkage;
    global UIAxes;
    if (~isempty(h_b1c1b2))
        delete(h_b1c1b2);
    end
    
    if (~isempty(h_b1c1b2))    
        delete(h_b1c2b2);
    end
    
    if (fiveLinkage.fk_nSol < 1)
        return;
    end

    if (fiveLinkage.current_configuration < 0)
        return;
    end

    upLineSpec  = '-mo';
    downLineSpec = ':mo';
    if (fiveLinkage.current_configuration == 0)
        upLineSpec  = ':mo';
        downLineSpec = '-mo';
    end

    h_b1c1b2 = plot(UIAxes, [fiveLinkage.B1(1) fiveLinkage.fk_up(1) fiveLinkage.B2(1)] , [fiveLinkage.B1(2) fiveLinkage.fk_up(2) fiveLinkage.B2(2)], upLineSpec);
    h_b1c1b2.PickableParts = 'none';
    
    if (fiveLinkage.fk_nSol == 2)
        h_b1c2b2 = plot(UIAxes, [fiveLinkage.B1(1) fiveLinkage.fk_down(1) fiveLinkage.B2(1)] , [fiveLinkage.B1(2) fiveLinkage.fk_down(2) fiveLinkage.B2(2)], downLineSpec);
        h_b1c2b2.PickableParts = 'none';
    end

    results = 0;                    
end

function  updateFkModeLabel()
    global UpLabel upModeLabel DownLabel downModeLabel UIModeKnob;
    global fiveLinkage;

    upModeLabel.Text = fiveLinkage.getWorkMode(1);
    downModeLabel.Text = fiveLinkage.getWorkMode(0);

    if (fiveLinkage.current_configuration == 1)
        UpLabel.BackgroundColor = [0.00,1.00,0.00];
        DownLabel.BackgroundColor = 'none';
        UIModeKnob.Value = upModeLabel.Text;
    elseif (fiveLinkage.current_configuration == 0)
        UpLabel.BackgroundColor = 'none';
        DownLabel.BackgroundColor = [0.00,1.00,0.00];
        UIModeKnob.Value = downModeLabel.Text;
    else
        UpLabel.BackgroundColor = 'none';
        DownLabel.BackgroundColor = 'none';
    end               
end