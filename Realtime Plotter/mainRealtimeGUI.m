function mainRealtimeGUI()
%% Main Realtime GUI
% This script creates the main GUI used to plot the realtime data being
% sent from the serial port.
%
% Script by erick nunez

%% clean up
close all;
%% Set up figure
fig1 = figure;
set(fig1, 'Units', 'Normalized', 'OuterPosition', [0,0, 1, 1]);
figAx = axes;
set(figAx, 'Units', 'normalized', 'OuterPosition', [0.03,0.05,0.7,0.95]);
hold on; grid on; xlim([-1.2,1.2]); ylim([-1.2,1.2]);

%% Variables
logLocation = 'no file';
collecting = false;
serialOpen = false;

%% UI
serialPanel = uipanel(fig1,'Units','normalized','Position',[0.75,0.8,0.1,0.15]);

serialCommButton = uicontrol(serialPanel,'Style','togglebutton');
serialCommButton.String = 'Serial Communication';
serialCommButton.Units = 'normalized';
serialCommButton.Position = [0.1,0.7,0.8,0.25];
serialCommButton.Callback = @openSerial;

loggingButton = uicontrol(serialPanel,'Style','togglebutton');
loggingButton.String = 'Log Data';
loggingButton.Units = 'normalized';
loggingButton.Position = [0.1,0.4,0.8,0.25];
loggingButton.Callback = @loggingData;

fileNameLabel = uicontrol(serialPanel,'Style','text');
fileNameLabel.String = logLocation;
fileNameLabel.Units = 'normalized';
fileNameLabel.Position = [0.1,0.1,0.8,0.25];

%% Graphing elements
initQE = 0;     initQS = 0;
initQDotE = 0;  initQDotS = 0;
goalQE = 0;     goalQS = 0;
goalQDotE = 0;  goalQDotS = 0;
initX = 0;      initY = 0;
initU = 0;      initV = 0;
goalX = 0;      goalY = 0;
goalU = 0;      goalV = 0;
forceX = 0;     forceY = 0;

%% Serial Object
baudrate = 115200;
port = 'COM24';
armSupport = serial(port);
armSupport.BaudRate = baudrate;

%% Main Loop
while 0
    if ~ishghandle(fig1)
        break
    end
    if collecting && serialOpen
        cla(figAx);
        serialData = fscanf(armSupport);
        temp = strsplit(serialData,'\t');
        time = 0.001*str2double(temp(1));
        forceX = str2double(temp(2));       forceY = str2double(temp(3));       % Forces
        initQE = str2double(temp(4));       initQS = str2double(temp(5));       % init Angle
        initQDotE = str2double(temp(6));    initQDotS = str2double(temp(7));    % init Ang Vel
        initX = str2double(temp(8));        initY = str2double(temp(9));        % init Pos
        initU = str2double(temp(10));       initV = str2double(temp(11));       % init Vel
        goalX = str2double(temp(12));       goalY = str2double(temp(13));       % goal Pos
        goalU = str2double(temp(14));       goalV = str2double(temp(15));       % goal Vel
        goalQE = str2double(temp(16));      goalQS = str2double(temp(17));      % goal Ang
        goalQDotE = str2double(temp(18));   goalQDotS = str2double(temp(19));   % goal Ang Vel
        write = str2double(temp(20));       % Write
        logObj.writeFrame([time,forceX,forceY,...
                           initQE,initQS,initQDotE,initQDotS,...
                           initX,initY,initU,initV,...
                           goalX,goalY,goalU,goalV,...
                           goalQE,goalQS,goalQDotE,goalQDotS,...
                           write]);  
       plot(initX,initY,'rs','Marksize',10);
       plot(goalX,goalY,'bo','Marksize',10);
    end
end

%% Clean up
delete(armSupport);

%% Callback functions

    function openSerial(src,event)
        if serialOpen
            serialOpen = false;
            fclose(armSupport);
        elseif ~serialOpen
            serialOpen = true;
            if ~isempty(instrfindall)
                delete(instrfindall)
            end
            fopen(armSupport);
            armSupport.ReadAsyncMode = 'continuous';
            readasync(s1);
        end
    end

    function loggingData(src,event)
        if collecting
            collecting = false;
            delete(logObj);
            logLocation = 'No file';
            fileNameLabel.String = logLocation;
        elseif ~collecting
            collecting = true;
            logObj = logFile;
            logObj.fileHeaders({'Time','ForceX','ForceY','initQE','initQS','initQDotE','initQDotS','initX',...
                'initY','initU','initV','goalX','goalY','goalU','goalV','goalQE','goalQS','goalQDotE','goalQDotS','Write'});
            logLocation = logObj.getName();
            fileNameLabel.String = logLocation;
        end
    end

end



