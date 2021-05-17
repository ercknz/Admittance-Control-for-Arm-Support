classdef armSupportComm < handle
%% Arm Support Communication class
% This class is setup to recieve data from the robotic arm support in order
% to log data and interact with virtual objects such as springs and walls.
% 
% Script by erick nunez

%% ------------------------------------------------------------------------
% Properties
%--------------------------------------------------------------------------
properties (Access = public)
    
end

properties (Access = private)
    serialObj
    baudRate = 115200
    rxPacketLen = 98
    dt = 0.008
    collectionTime
    portName
    rawBytes = nan(1,rxPacketLen)
    frameData = nan(1,23)
    fileName    = 'armSupportLog'
    fileExt     = '.xlsx'
    folderPath  = './Logs/'
end

%% ------------------------------------------------------------------------
% Methods
%--------------------------------------------------------------------------
methods
    function obj = armSupportComm(port)
        obj.portName = port;
    end
        
    function CommStart(obj, collectionTime)
        obj.collectionTime = collectionTime;
        obj.serialObj = serialPort(obj.portName, obj.baudRate);
    end
    
    function CommStop(obj)
        instrreset;
        obj.collectionTime = nan;
        obj.serialObj = nan;
    end
    
    function numOfBytes = CommBytesAvailable(obj)
        numOfBytes = obj.serialObj.NumBytesAvailable;
    end
    
    function CommUpdate(obj)
        if obj.serialObj.NumBytesAvailable >= obj.rxPacketLen
           obj.rawBytes = read(obj.serialObj,obj.rxPacketLen,'uint8');
           % Total Time
           obj.frameData(1,1) = 0.001 * typecast(uint8(obj.rawBytes(1,5:8)),'uint32');
           % Loop Time
           obj.frameData(1,23) = typecast(uint8(obj.rawBytes(1,93:96)),'uint32');
        end
    end
    
    function CommConfig(obj)
        
    end
    
    function CommModify(obj)
        
    end
    
    function CreateSpring(obj)
        
    end
    
    function AddWall(obj)
        
    end
end

end