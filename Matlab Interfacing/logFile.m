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
    lastFrame = (0);
    frameLength
    
    data
    logHeaders
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
    end
        
    function obj = createLog(obj)
        if isempty(obj.fullFileName)
            obj.fileTime = datestr(now,'mmddyyHHMM');
            obj.fullFileName = [obj.folderPath, obj.fileName, obj.fileTime, obj.fileExt];
        end
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
    
    function obj = deleteLog(obj)
        if ~isempty(obj.fullFileName)
            obj.fileTime = [];
            obj.fullFileName = [];
        end
    end
    
    function fullFileName = getName(obj)
        fullFileName = obj.fullFileName;
    end
    
    function addFrame(obj,frame)
        obj.lastFrame = frame;
        obj.data(obj.lineNum,:) = frame;
        obj.lineNum = obj.lineNum + 1;
    end
    
    function tempData(obj,secs)
        obj.data = nan(secs * 100, obj.frameLength);
    end
    
    function saveLog(obj)
        if ~isempty(obj.logHeaders)
            writecell(obj.logHeaders,obj.fullFileName,'Range','A1');
        end
        if ~isempty(obj.data)
           writematrix(obj.data,obj.fullFileName,'Range','B1'); 
        end
    end
    
    function setHeaders(obj,headers)
        obj.logHeaders = headers;
        obj.frameLength = length(headers);
    end
end

end