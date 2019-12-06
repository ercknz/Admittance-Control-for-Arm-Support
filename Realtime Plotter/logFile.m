classdef logFile < handle
%% Data to Log File Function
% This function writes the data collected per frame to the a created csv
% file.
% 
% Script by erick nunez

%% ------------------------------------------------------------------------
% Properties
%--------------------------------------------------------------------------
properties (Access = public)
    lastFrame
    frameLength
end

properties (Access = private)
    fileName    = 'armSupportLog'
    fileTime
    fileExt     = '.xlsx'
    folderPath  = './Logs/'
    lineNum = 1
    fullFileName
end

%% ------------------------------------------------------------------------
% Methods
%--------------------------------------------------------------------------
methods
    function obj = logFile()
        if ~exist(obj.folderPath,'dir')
            mkdir(obj.folderPath)
        end
        obj.fileTime = datestr(now,'mmddyyHHMM');
        obj.fullFileName = [obj.folderPath, obj.fileName, obj.fileTime, obj.fileExt];
    end
    function obj = fileHeaders(obj,headers)
        if obj.lineNum == 1
            obj.frameLength = length(headers);
            obj.lineNum = obj.lineNum + 1;
        end
        writecell(headers,obj.fullFileName,'Range','A1')
    end
    function obj = writeFrame(obj,frame)
        obj.lastFrame = frame;
        if obj.lineNum == 1
            obj.frameLength = length(frame);
            writematrix(frame,obj.fullFileName,'Range','A2');
            obj.lineNum = 3;
        else
            writematrix(frame,obj.fullFileName,'Range',['A',num2str(obj.lineNum)]);
            obj.lineNum = obj.lineNum + 1;
        end
    end
    function fullFileName = getName(obj)
        fullFileName = obj.fullFileName;
    end
end

end