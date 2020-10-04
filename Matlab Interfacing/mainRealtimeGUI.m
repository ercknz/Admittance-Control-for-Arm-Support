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
    'Units', 'Normalized',...
    'OuterPosition', [0,0, 1, 1]);
figAx = axes;
set(figAx, 'Units', 'normalized',...
    'OuterPosition', [0.03,0.05,0.7,0.95],...
    'XLim',[-1.2,1.2],...
    'YLim',[-1.2,1.2]);
hold on; grid on;
% Initializing graphics
[x1, y1] = endLink1(0);
h.presL1 = line([0, x1],[0, y1],'Color','green','LineWidth',3,'Parent',figAx);
[x2, y2] = endLink2(0, 0);
h.presL2 = line([x1, x2],[y1, y2],'Color','green','LineWidth',3,'Parent',figAx);
[x, y, u, v] = armSupportFKine(0, 0, 0, 0);
h.presMass = plot(x,y,'g*','MarkerSize',10,'Parent',figAx);
h.presVel = quiver(figAx, x, y, u, v);
% Goal Robot
h.goalMass = plot(1,0,'ro','MarkerSize',10,'Parent',figAx);
h.goalVel = quiver(figAx, 1, 0, 0, 0);
[qs, qe, ~, ~] = armSupportIKine(1, 0, 0, 0);
[x1, y1] = endLink1(qs);
h.goalL1 = line([0, x1],[0, y1],'Color','red','LineWidth',3,'Parent',figAx);
[x2, y2] = endLink2(qs, qe);
h.goalL2 = line([x1, x2],[y1, y2],'Color','red','LineWidth',3,'Parent',figAx);

%% Variables
logLocation = 'no file';
logObj = logFile;
armSupport = [];
graphing = true;

%% UI
serialPanel = uipanel(fig1,'Units','normalized','Position',[0.75,0.75,0.1,0.25]);

serialCommButton = uicontrol(serialPanel,'Style','togglebutton');
serialCommButton.String = 'Serial Communication';
serialCommButton.Units = 'normalized';
serialCommButton.Position = [0.1,0.725,0.8,0.225];

loggingButton = uicontrol(serialPanel,'Style','togglebutton');
loggingButton.String = 'Log Data';
loggingButton.Units = 'normalized';
loggingButton.Position = [0.1,0.5,0.8,0.225];

graphingButton = uicontrol(serialPanel,'Style','togglebutton');
graphingButton.String = 'Plot Off';
graphingButton.Units = 'normalized';
graphingButton.Position = [0.1,0.275,0.8,0.225];

fileNameLabel = uicontrol(serialPanel,'Style','text');
fileNameLabel.String = logLocation;
fileNameLabel.Units = 'normalized';
fileNameLabel.Position = [0.1,0.05,0.8,0.225];

%% Set callbacks
set(fig1,'CloseRequestFcn',@uiCloseFigureFcn);
serialCommButton.Callback = @openSerial;
loggingButton.Callback = @loggingData;
graphingButton.Callback = @graphData;

%% Callback functions
    function armSupportCallback(src, ~)
        serialData = readline(src);
        temp = strsplit(serialData,'\t');
        time = 0.001*str2double(temp(1));
        forceX = str2double(temp(2));       forceY = str2double(temp(3));       % Forces
        presQS = str2double(temp(4));       presQE = str2double(temp(5));       % init Angle
        presQdotS = str2double(temp(6));    presQdotE = str2double(temp(7));    % init Ang Vel
        goalX = str2double(temp(8));        goalY = str2double(temp(9));        % goal Pos
        goalXdot = str2double(temp(10));    goalYdot = str2double(temp(11));    % goal Vel
        goalQS = str2double(temp(12));      goalQE = str2double(temp(13));      % goal Ang
        goalQdotS = str2double(temp(14));   goalQdotE = str2double(temp(15));   % goal Ang Vel
        loopTime = str2double(temp(17));    % Loop time
        mWrite = str2double(temp(16));      % Write
        if ~isempty(logObj.getName())
            logObj.writeFrame([time,...
                forceX,forceY,...
                presQS,presQE,presQdotS,presQdotE,...
                goalX,goalY,goalXdot,goalYdot,...
                goalQS,goalQE,goalQdotS,goalQdotE,...
                mWrite, loopTime]);
        end
        if graphing
            % pres Robot
            [X1, Y1] = endLink1(presQS);
            set(h.presL1,'XData',[0,X1],'YData',[0,Y1]);
            [X2, Y2] = endLink2(presQS, presQE);
            set(h.presL2,'XData',[X1,X2],'YData',[Y1,Y2]);
            [X, Y, U, V] = armSupportFKine(presQS, presQE, presQdotS, presQdotE);
            set(h.presMass,'XData',X,'YData',Y);
            set(h.presVel,'XData',X,'YData',Y,'UData',U,'VData',V);
            % Goal Robot
            set(h.goalMass,'XData',goalX,'YData',goalY);
            set(h.goalVel,'XData',goalX,'YData',goalY,'UData',goalXdot,'VData',goalYdot)
            [goalQS, goalQE, ~, ~] = armSupportIKine(goalX, goalY, goalXdot, goalYdot);
            [X1, Y1] = endLink1(goalQS);
            set(h.goalL1,'XData',[0, X1],'YData',[0, Y1]);
            [X2, Y2] = endLink2(goalQS, goalQE);
            set(h.goalL2,'XData',[X1, X2],'YData',[Y1, Y2]);
        end
    end

    function openSerial(hObject, eventdata)
        if ~isempty(armSupport)
            configureCallback(armSupport, 'off');
            armSupport.delete();
            armSupport = [];
        else
            armSupport = serialport('COM24',115200);
            configureTerminator(armSupport, 'CR/LF');
            flush(armSupport)
            configureCallback(armSupport, 'terminator',@armSupportCallback)
        end
    end

    function loggingData(hObject,eventdata)
        if ~isempty(logObj.getName())
            logObj.deleteLog()
            logLocation = 'No file';
            fileNameLabel.String = logLocation;
        else
            logObj.createLog();
            logObj.fileHeaders({'Time','ForceX','ForceY','presQS','presQE','presQdotS','presQdotE',...
                'goalX','goalY','goalXdot','goalYdot','goalQS','goalQE','goalQdotS','goalQdotE','Write','loopTime'});
            logLocation = logObj.getName();
            fileNameLabel.String = logLocation;
        end
    end

    function graphData(hObject, eventdata)
        if graphing
            graphing = false;
        else
            graphing = true;
        end
    end

    function uiCloseFigureFcn(hObject, eventdata)
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



