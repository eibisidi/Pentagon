clear;
close all;

global fiveLinkage;
fiveLinkage = RRRRR;
fiveLinkage.r = [1.2; 1.0; 0.8];
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

 %draw singularity curve
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
%Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta1SliderValueChanged(~, event)
    global theta2Slider;
    global fiveLinkage;
    fiveLinkage.theta = [event.Value; theta2Slider.Value;];
    handleThetaChanged();
end

function theta2SliderValueChanged(~, event)
    global theta1Slider;
    global fiveLinkage;
    fiveLinkage.theta = [ theta1Slider.Value; event.Value];
    handleThetaChanged();
end

function [outputArg1,outputArg2] = AxesMouseClicked(Object, eventData)
 cp = Object.CurrentPoint(1,:);
 disp(cp);
end

function results = handleThetaChanged()
    global fiveLinkage;
    fiveLinkage.b1 = [fiveLinkage.r(1) * cosd(fiveLinkage.theta(1)) - fiveLinkage.r(3); fiveLinkage.r(1) * sind(fiveLinkage.theta(1))];
    fiveLinkage.b2 = [fiveLinkage.r(1) * cosd(fiveLinkage.theta(2)) + fiveLinkage.r(3); fiveLinkage.r(1) * sind(fiveLinkage.theta(2))];
    drawActiveLink();

    %do forward kinematics
    [fiveLinkage.fk_nSol, fiveLinkage.fk_up, fiveLinkage.fk_down] = forwardKinematics(fiveLinkage.r, fiveLinkage.theta);

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
        
function crossChar = cross2char(crossValue)
     if (crossValue < 1E-6)
        crossChar = '-';
     elseif (crossValue > 1E-6)
        crossChar = '+';
    else
        crossChar = 'c';
    end
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
    h_a1b1 = plot(UIAxes, [fiveLinkage.A1(1) fiveLinkage.b1(1)] , [fiveLinkage.A1(2) fiveLinkage.b1(2)], '-go');
    h_a1b1.PickableParts = 'none';
    h_a2b2 = plot(UIAxes, [fiveLinkage.A2(1) fiveLinkage.b2(1)] , [fiveLinkage.A2(2) fiveLinkage.b2(2)], '-go');
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

    h_b1c1b2 = plot(UIAxes, [fiveLinkage.b1(1) fiveLinkage.fk_up(1) fiveLinkage.b2(1)] , [fiveLinkage.b1(2) fiveLinkage.fk_up(2) fiveLinkage.b2(2)], upLineSpec);
    h_b1c1b2.PickableParts = 'none';
    
    if (fiveLinkage.fk_nSol == 2)
        h_b1c2b2 = plot(UIAxes, [fiveLinkage.b1(1) fiveLinkage.fk_down(1) fiveLinkage.b2(1)] , [fiveLinkage.b1(2) fiveLinkage.fk_down(2) fiveLinkage.b2(2)], downLineSpec);
        h_b1c2b2.PickableParts = 'none';
    end

    results = 0;                    
end

function results = updateFkModeLabel()
    global UpLabel upModeLabel DownLabel downModeLabel;
    global fiveLinkage;
    if (fiveLinkage.fk_nSol < 1 || fiveLinkage.current_configuration < 0)
        upModeLabel.Text = 'xx';
        downModeLabel.Text = 'xx';
        UpLabel.BackgroundColor = 'none';
        DownLabel.BackgroundColor = 'none';
        return;
    end

    A1C1 = fiveLinkage.fk_up - fiveLinkage.A1; %vector A1P
    A1B1 = fiveLinkage.b1 - fiveLinkage.A1;    %vector A1B1
    leftCross = A1C1(1) * A1B1(2) - A1B1(1) * A1C1(2);

    A2C1 = fiveLinkage.fk_up - fiveLinkage.A2;
    A2B2 = fiveLinkage.b2 - fiveLinkage.A2;
    rightCross = A2C1(1) * A2B2(2) - A2B2(1) * A2C1(2);

    upModeLabel.Text = [cross2char(leftCross) , cross2char(rightCross)];

    A1C2 = fiveLinkage.fk_down - fiveLinkage.A1; %vector A1P
    A1B1 = fiveLinkage.b1 - fiveLinkage.A1;    %vector A1B1
    leftCross = A1C2(1) * A1B1(2) - A1B1(1) * A1C2(2);

    A2C2 = fiveLinkage.fk_down - fiveLinkage.A2;
    A2B2 = fiveLinkage.b2 - fiveLinkage.A2;
    rightCross = A2C2(1) * A2B2(2) - A2B2(1) * A2C2(1);

    downModeLabel.Text = [cross2char(leftCross) , cross2char(rightCross)];

    if (fiveLinkage.current_configuration == 1)
        UpLabel.BackgroundColor = [0.00,1.00,0.00];
        DownLabel.BackgroundColor = 'none';
    else
        UpLabel.BackgroundColor = 'none';
        DownLabel.BackgroundColor = [0.00,1.00,0.00];
    end

    results = 0;                    
end