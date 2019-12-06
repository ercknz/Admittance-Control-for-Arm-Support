classdef armSupportSerial < handle
%% Arm Support Serial Class
% This object encapsulates the serial port of the arm support.
% 
% Script by erick nunez

%% ------------------------------------------------------------------------
% Properties
%--------------------------------------------------------------------------
properties (Access = public)
    armSupport
    frame
end

properties (Access = private)
    baudrate = 115200;
    port = 'COM24'
end

%% ------------------------------------------------------------------------
% Methods
%--------------------------------------------------------------------------
methods
    function obj = armSupportSerial()
        obj.armSupport = serial(obj.port);
        obj.armSupport.BaudRate = obj.baudrate;
    end
    function open(obj)
        if ~isempty(instrfindall)
            delete(instrfindall)
        end
        fopen(obj.armSupport);
        obj.armSupport.ReadAsyncMode = 'continuous';
        readasync(s1);
    end
    function obj = readFrame(obj)
        serialData = fscanf(obj.armSupport);
        obj.frame = strsplit(serialData,'\t');
    end
    function close(obj)
        fclose(obj.armSupport);
        delete(obj.armSupport);
    end
end

end
