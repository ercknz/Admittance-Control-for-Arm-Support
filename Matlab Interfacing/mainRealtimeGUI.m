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
    'OuterPosition', [0.0,0.1,0.33,0.5],...
    'XLim',[-1.2,1.2],...
    'YLim',[-1.2,1.2]);
hold on; grid on;
% Initializing object
armSupport = armSupportComm();
guidata(fig1,armSupport);

%% Variables
IncomingBytes = uint8(ones(1,armSupport.rxPacketLen)* 255);
BytesToDisplay = num2str(IncomingBytes);
Data.eTime = 0.0;          
Data.slot1 = 0.0;       Data.slot2 = 0.0;       Data.slot3 = 0.0;
Data.slot4 = 0.0;       Data.slot5 = 0.0;       Data.slot6 = 0.0;
Data.slot7 = 0.0;       Data.slot8 = 0.0;       Data.slot9 = 0.0;
Data.slot10 = 0.0;      Data.slot11 = 0.0;      Data.slot12 = 0.0;
Data.slot13 = 0.0;      Data.slot14 = 0.0;      Data.slot15 = 0.0;
Data.slot16 = 0.0;      Data.slot17 = 0.0;      Data.slot18 = 0.0;
Data.slot19 = 0.0;      Data.slot20 = 0.0;      Data.slot21 = 0.0;
Data.lTime = 0.0;
dataForLabel = sprintf('elapsed Time = %0.3f\n%0.3f %0.3f %0.3f\n%0.3f %0.3f %0.3f\n%0.3f %0.3f %0.3f\n%0.3f %0.3f %0.3f\n%0.3f %0.3f %0.3f\n%0.3f %0.3f %0.3f\n%0.3f %0.3f %0.3f\nLoopTime = %0.3f',...
    Data.eTime,...
    Data.slot1,  Data.slot2,  Data.slot3,...
    Data.slot4,  Data.slot5,  Data.slot6,...
    Data.slot7,  Data.slot8,  Data.slot9,...
    Data.slot10, Data.slot11, Data.slot12,...
    Data.slot13, Data.slot14, Data.slot15,...
    Data.slot16, Data.slot17, Data.slot18,...
    Data.slot19, Data.slot20, Data.slot21,...
    Data.lTime);

%% UI
ButtonPanel = uipanel(fig1,'Units','normalized','Position',[0.75,0.65,0.1,0.2]);

serialCommButton = uicontrol(ButtonPanel,'Style','togglebutton');
serialCommButton.String = 'Serial Communication';
serialCommButton.Units = 'normalized';
serialCommButton.Position = [0.1,0.5,0.8,0.375];
serialCommButton.BackgroundColor = [0.85,0.33,0.10];

saveConfigButton = uicontrol(ButtonPanel,'Style','pushbutton');
saveConfigButton.String = 'Save Configuration';
saveConfigButton.Units = 'normalized';
saveConfigButton.Position = [0.1,0.1,0.8,0.375];

SlidersPanel = uipanel(fig1,'Units','normalized','Position',[0.33,0.1,0.33,0.5]);

MxyLabel = uicontrol(SlidersPanel,'Style','text','String','MassXY');
MxyLabel.Units = 'normalized';
MxyLabel.Position = [0.1,0.8,0.15,0.1];
MxySlider = uicontrol(SlidersPanel,'Style','slider','Value',0.1);
MxySlider.Units = 'normalized';
MxySlider.Position = [0.1,0.125,0.15,0.7];
MxySlider.Min = 0.1;
MxySlider.Max = 50;
MxySlider.SliderStep = [0.01 0.1];
MxySendValue = uicontrol(SlidersPanel,'Style','pushbutton','String',num2str(MxySlider.Value));
MxySendValue.Units = 'normalized';
MxySendValue.Position = [0.1,0.05,0.15,0.05];

MzLabel = uicontrol(SlidersPanel,'Style','text','String','MassZ');
MzLabel.Units = 'normalized';
MzLabel.Position = [0.26,0.8,0.15,0.1];
MzSlider = uicontrol(SlidersPanel,'Style','slider','Value',0.1);
MzSlider.Units = 'normalized';
MzSlider.Position = [0.26,0.125,0.15,0.7];
MzSlider.Min = 0.1;
MzSlider.Max = 50;
MzSlider.SliderStep = [0.01 0.1];
MzSendValue = uicontrol(SlidersPanel,'Style','pushbutton','String',num2str(MzSlider.Value));
MzSendValue.Units = 'normalized';
MzSendValue.Position = [0.26,0.05,0.15,0.05];

BxyLabel = uicontrol(SlidersPanel,'Style','text','String','DampingXY');
BxyLabel.Units = 'normalized';
BxyLabel.Position = [0.42,0.8,0.15,0.1];
BxySlider = uicontrol(SlidersPanel,'Style','slider','Value',0.1);
BxySlider.Units = 'normalized';
BxySlider.Position = [0.42,0.125,0.15,0.7];
BxySlider.Min = 0.1;
BxySlider.Max = 50;
BxySlider.SliderStep = [0.01 0.1];
BxySendValue = uicontrol(SlidersPanel,'Style','pushbutton','String',num2str(BxySlider.Value));
BxySendValue.Units = 'normalized';
BxySendValue.Position = [0.42,0.05,0.15,0.05];

BzLabel = uicontrol(SlidersPanel,'Style','text','String','DampingZ');
BzLabel.Units = 'normalized';
BzLabel.Position = [0.58,0.8,0.15,0.1];
BzSlider = uicontrol(SlidersPanel,'Style','slider','Value',0.1);
BzSlider.Units = 'normalized';
BzSlider.Position = [0.58,0.125,0.15,0.7];
BzSlider.Min = 0.1;
BzSlider.Max = 50;
BzSlider.SliderStep = [0.01 0.1];
BzSendValue = uicontrol(SlidersPanel,'Style','pushbutton','String',num2str(BzSlider.Value));
BzSendValue.Units = 'normalized';
BzSendValue.Position = [0.58,0.05,0.15,0.05];

SpringLabel = uicontrol(SlidersPanel,'Style','text','String',{'Spring';'Compensation'});
SpringLabel.Units = 'normalized';
SpringLabel.Position = [0.74,0.8,0.15,0.1];
SpringSlider = uicontrol(SlidersPanel,'Style','slider','Value',0.0);
SpringSlider.Units = 'normalized';
SpringSlider.Position = [0.74,0.125,0.15,0.7];
SpringSlider.Min = 0.0;
SpringSlider.Max = 1.0;
SpringSlider.SliderStep = [0.01 0.1];
SpringSendValue = uicontrol(SlidersPanel,'Style','pushbutton','String',num2str(SpringSlider.Value));
SpringSendValue.Units = 'normalized';
SpringSendValue.Position = [0.74,0.05,0.15,0.05];

DataPanel = uipanel(fig1,'Units','normalized','Position',[0.05,0.65,0.61,0.25]);
BytesText = uicontrol(DataPanel,'Style','text','String',BytesToDisplay);
BytesText.Units = 'normalized';
BytesText.Position = [0.1,0.1,0.4,0.8];
DataText = uicontrol(DataPanel,'Style','text','String',dataForLabel);
DataText.Units = 'normalized';
DataText.Position = [0.5,0.1,0.4,0.8];
DataText.FontSize = 12;

CurrentConfig = sprintf('{\n\t"Mxy":"%0.3f",\n\t"Mz":"%0.3f",\n\t"Bxy":"%0.3f",\n\t"Bz":"%0.3f",\n\t"Spring":"%0.3f"\n}'...
                        ,str2double(MxySendValue.String),str2double(MzSendValue.String)...
                        ,str2double(BxySendValue.String),str2double(BzSendValue.String)...
                        ,str2double(SpringSendValue.String));
CurrentConfigPanel = uipanel(fig1,'Units','normalized','Position',[0.67,0.1,0.32,0.5]);
CurrentConfigText = uicontrol(CurrentConfigPanel,'Style','text','String',CurrentConfig);
CurrentConfigText.HorizontalAlignment = 'left';
CurrentConfigText.FontSize = 12;
CurrentConfigText.Units = 'normalized';
CurrentConfigText.Position = [0.1,0.1,0.8,0.8];

%% Set callbacks
set(fig1,'CloseRequestFcn',@ uiCloseFigureFcn);
serialCommButton.Callback = @(serialCommButton,event) ToggleCommUI(serialCommButton,armSupport);
saveConfigButton.Callback = @(saveConfigButton,event) SaveConfiguration(MxySendValue,MzSendValue,BxySendValue,BzSendValue,SpringSendValue);

MxySlider.Callback = @(MxySlider,event) ChangeMassXY(MxySlider,MxySendValue);
MxySendValue.Callback = @(MxySendValue,event) SendMxyValue(MxySendValue,armSupport,MzSendValue,BxySendValue,BzSendValue,SpringSendValue,CurrentConfigText);

MzSlider.Callback = @(MzSlider,event) ChangeMassZ(MzSlider,MzSendValue);
MzSendValue.Callback = @(MzSendValue,event) SendMzValue(MzSendValue,armSupport,MxySendValue,BxySendValue,BzSendValue,SpringSendValue,CurrentConfigText);

BxySlider.Callback = @(BxySlider,event) ChangeDampingXY(BxySlider,BxySendValue);
BxySendValue.Callback = @(BxySendValue,event) SendBxyValue(BxySendValue,armSupport,MxySendValue,MzSendValue,BzSendValue,SpringSendValue,CurrentConfigText);

BzSlider.Callback = @(BzSlider,event) ChangeDampingZ(BzSlider,BzSendValue);
BzSendValue.Callback = @(BzSendValue,event) SendBzValue(BzSendValue,armSupport,MxySendValue,MzSendValue,BxySendValue,SpringSendValue,CurrentConfigText);

SpringSlider.Callback = @(SpringSlider,event) ChangeSpringComp(SpringSlider,SpringSendValue);
SpringSendValue.Callback = @(SpringSendValue,event) SendSpringValue(SpringSendValue,armSupport,MxySendValue,MzSendValue,BxySendValue,BzSendValue,CurrentConfigText);


end

%% Callback functionns
function ToggleCommUI(button, commObj)
    if commObj.IsCommOpen()
        button.BackgroundColor = [0.85,0.33,0.10];
        commObj.CommStop();
    else
        button.BackgroundColor = [0.39,0.83,0.07];
        commObj.CommStart();
    end
end

function SaveConfiguration(mxy, mz, bxy, bz, spring)
    fileID = fopen('config.txt','w');
    fprintf(fileID,'{\n\t"Mxy":"%0.3f",\n\t"Mz":"%0.3f",\n\t"Bxy":"%0.3f",\n\t"Bz":"%0.3f",\n\t"Spring":"%0.3f"\n}'...
                   ,str2double(mxy.String),str2double(mz.String)...
                   ,str2double(bxy.String),str2double(bz.String)...
                   ,str2double(spring.String));
    fclose(fileID);
end

function UpdateConfig(config, mxy, mz, bxy, bz, spring)
    config.String = sprintf('{\n\t"Mxy":"%0.3f",\n\t"Mz":"%0.3f",\n\t"Bxy":"%0.3f",\n\t"Bz":"%0.3f",\n\t"Spring":"%0.3f"\n}'...
                            ,str2double(mxy.String),str2double(mz.String)...
                            ,str2double(bxy.String),str2double(bz.String)...
                            ,str2double(spring.String));
end

function ChangeMassXY(slider,text)
    text.String = num2str(slider.Value);
end

function SendMxyValue(button, commObj, mz, bxy, bz, spring, text)
    if commObj.IsCommOpen()
        commObj.CommSendMassXY(str2double(button.String));
    end
    UpdateConfig(text, button, mz, bxy, bz, spring);
end

function ChangeMassZ(slider, text)
    text.String = num2str(slider.Value);
end

function SendMzValue(button, commObj, mxy, bxy, bz, spring, text)
    if commObj.IsCommOpen()
        commObj.CommSendMassZ(str2double(button.String));
    end
    UpdateConfig(text, mxy, button, bxy, bz, spring);
end

function ChangeDampingXY(slider, text)
    text.String = num2str(slider.Value);
end

function SendBxyValue(button, commObj, mxy, mz, bz, spring, text)
    if commObj.IsCommOpen()
        commObj.CommSendDampingXY(str2double(button.String));
    end
    UpdateConfig(text, mxy, mz, button, bz, spring);
end

function ChangeDampingZ(slider, text)
    text.String = num2str(slider.Value);
end

function SendBzValue(button, commObj, mxy, mz, bxy, spring, text)
    if commObj.IsCommOpen()
        commObj.CommSendDampingZ(str2double(button.String));
    end
    UpdateConfig(text, mxy, mz, bxy, button, spring);
end

function ChangeSpringComp(slider, text)
    text.String = num2str(slider.Value);
end

function SendSpringValue(button, commObj, mxy, mz, bxy, bz, text)
    if commObj.IsCommOpen()
        commObj.CommSendSpring(str2double(button.String));
    end
    UpdateConfig(text, mxy, mz, bxy, bz, button);
end

function SendExtForce(hObject, eventdata)

end

function uiCloseFigureFcn(hObject, eventdata)
    armSupport = guidata(hObject);
    if armSupport.IsCommOpen()
        armSupport.CommStop();
    end
    delete(gcf);
end

