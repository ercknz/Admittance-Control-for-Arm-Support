function mainRealtimeGUI()
%% Main Realtime GUI
% This script creates the main GUI used to plot the realtime data being
% sent from the serial port.
%
% Script by erick nunez

%% clean up
close all; instrreset;
%% Set up figure
fig1 = figure;
set(fig1,'Name', 'Realtime Plotter for Arm Support',...
    'NumberTitle','off',...
    'CloseRequestFcn',@uiCloseFigureFcn,...
    'Units', 'Normalized',...
    'OuterPosition', [0,0, 1, 1]);
figAx = axes;
set(figAx, 'Units', 'normalized',...
    'OuterPosition', [0.03,0.05,0.7,0.95],...
    'XLim',[-1.2,1.2],...
    'YLim',[-1.2,1.2]);
hold on; grid on;

%% Variables
logLocation = 'no file';
logObj = logFile;
armSupport = [];
forcePlot = false;

%% UI
serialPanel = uipanel(fig1,'Units','normalized','Position',[0.75,0.75,0.1,0.25]);

serialCommButton = uicontrol(serialPanel,'Style','togglebutton');
serialCommButton.String = 'Serial Communication';
serialCommButton.Units = 'normalized';
serialCommButton.Position = [0.1,0.725,0.8,0.225];
serialCommButton.Callback = @openSerial;

loggingButton = uicontrol(serialPanel,'Style','togglebutton');
loggingButton.String = 'Log Data';
loggingButton.Units = 'normalized';
loggingButton.Position = [0.1,0.5,0.8,0.225];
loggingButton.Callback = @loggingData;

forceButton = uicontrol(serialPanel,'Style','togglebutton');
forceButton.String = 'Force Plot';
forceButton.Units = 'normalized';
forceButton.Position = [0.1,0.275,0.8,0.225];
forceButton.Callback = @forceData;

fileNameLabel = uicontrol(serialPanel,'Style','text');
fileNameLabel.String = logLocation;
fileNameLabel.Units = 'normalized';
fileNameLabel.Position = [0.1,0.05,0.8,0.225];

%% Callback functions
    function armSupportCallback(src,~)
        cla(figAx);
        serialData = readline(src);
        temp = strsplit(serialData,'\t');
        time = 0.001*str2double(temp(1));
        rawForceX = str2double(temp(2));    rawForceY = str2double(temp(3));    % Raw Forces
        forceX = str2double(temp(4));       forceY = str2double(temp(5));       % Forces
        presQS = str2double(temp(6));       presQE = str2double(temp(7));       % init Angle
        presQdotS = str2double(temp(8));    presQdotE = str2double(temp(9));    % init Ang Vel
        initX = str2double(temp(10));       initY = str2double(temp(11));       % init Pos
        initXdot = str2double(temp(12));    initYdot = str2double(temp(13));    % init Vel
        goalX = str2double(temp(14));       goalY = str2double(temp(15));       % goal Pos
        goalXdot = str2double(temp(16));    goalYdot = str2double(temp(17));    % goal Vel
        goalQS = str2double(temp(18));      goalQE = str2double(temp(19));      % goal Ang
        goalQdotS = str2double(temp(20));   goalQdotE = str2double(temp(21));   % goal Ang Vel
        mWrite = str2double(temp(22));       % Write
        if ~isempty(logObj.getName())
            logObj.writeFrame([time,rawForceX,rawForceY,...
                forceX,forceY,...
                presQS,presQE,presQdotS,presQdotE,...
                initX,initY,initXdot,initYdot,...
                goalX,goalY,goalXdot,goalYdot,...
                goalQS,goalQE,goalQdotS,goalQdotE,...
                mWrite]);
        end
        if forcePlot
            [globalFx, globalFy] = sensorOrientation(rawForceX, rawForceY, presQS, presQE);
            quiver(figAx, 0, 0, 2*globalFx, 2*globalFy, 'Color', '#A2142F', 'LineWidth', 3);
            quiver(figAx, 0, 0, forceX, forceY, 'Color', '#77AC30', 'LineWidth', 3);
        else
            % pres Robot
            [presL1.X, presL1.Y] = endLink1(presQS);
            line([0, presL1.X],[0, presL1.Y],'Color','green','LineWidth',3,'Parent',figAx);
            [presL2.X, presL2.Y] = endLink2(presQS, presQE);
            line([presL1.X, presL2.X],[presL1.Y, presL2.Y],'Color','green','LineWidth',3,'Parent',figAx);
            [presX, presY, presXdot, presYdot] = armSupportFKine(presQS, presQE, presQdotS, presQdotE);
            plot(presX,presY,'g*','MarkerSize',10,'Parent',figAx);
            quiver(figAx, presX, presY, presXdot, presYdot);
            % Goal Robot
            plot(goalX,goalY,'ro','MarkerSize',10,'Parent',figAx);
            quiver(figAx, goalX, goalY, goalXdot, goalYdot);
            [goalQS, goalQE, ~, ~] = armSupportIKine(goalX, goalY, goalXdot, goalYdot);
            [goalL1.X, goalL1.Y] = endLink1(goalQS);
            line([0, goalL1.X],[0, goalL1.Y],'Color','red','LineWidth',3,'Parent',figAx);
            [goalL2.X, goalL2.Y] = endLink2(goalQS, goalQE);
            line([goalL1.X, goalL2.X],[goalL1.Y, goalL2.Y],'Color','red','LineWidth',3,'Parent',figAx);
        end
        drawnow limitrate;
    end

    function openSerial(src,~)
        if ~isempty(armSupport)
            configureCallback(armSupport, "off");
            armSupport.delete();
            armSupport = [];
        else
            armSupport = serialport("COM24",115200);
            configureTerminator(armSupport, "CR/LF");
            flush(armSupport)
            configureCallback(armSupport, "terminator",@armSupportCallback)
        end
    end

    function loggingData(src,~)
        if ~isempty(logObj.getName())
            logObj.deleteLog()
            logLocation = 'No file';
            fileNameLabel.String = logLocation;
        else
            logObj.createLog();
            logObj.fileHeaders({'Time','rawForceX','rawForceY','ForceX','ForceY','presQS','presQE','presQdotS','presQdotE','initX',...
                'initY','initXdot','initYdot','goalX','goalY','goalXdot','goalYdot','goalQS','goalQE','goalQdotS','goalQdotE','Write'});
            logLocation = logObj.getName();
            fileNameLabel.String = logLocation;
        end
    end

    function forceData(src,~)
        if forcePlot
            forcePlot = false;
        else
            forcePlot = true;
        end
    end

    function uiCloseFigureFcn(src,callbackdata)
        if ~isempty(armSupport)
            configureCallback(armSupport, "off");
            armSupport.delete();
            armSupport = [];
        end
        delete(gcf)
    end

%% Other functions
    function [X,Y] = endLink1(qS)
        L1 = 0.510;
        X = L1 * cos(qS);
        Y = L1 * sin(qS);
    end

    function [X,Y] = endLink2(qS, qE)
        L1 = 0.510;
        L2 = 0.505;
        X = L1 * cos(qS) + L2 * cos(qS+qE);
        Y = L1 * sin(qS) + L2 * sin(qS+qE);
    end

end



