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
        serialObj
        baudRate = 115200
        dt = 0.008
        portName = 'COM28'
        rawBytes
        frameData = nan(1,23)
        fileName    = 'armSupportLog'
        fileExt     = '.xlsx'
        folderPath  = './Logs/'
        configFileExt = '.txt'
        configFileName = 'armSupportConfig'
        
        modHeader = uint8([150, 10, 10, 96])
        configHeader = unit([150, 0, 69, 8])
        txPacketLen = 39
        
        rxHeader = uint8([170, 8, 69, 0])
        rxPacketLen = 98
    end
    
    properties (Access = private)
        CommOpen = false;
    end
    
    %% ------------------------------------------------------------------------
    % Methods
    %--------------------------------------------------------------------------
    methods
        function obj = armSupportComm()
            obj.rawBytes = nan(1,obj.rxPacketLen);
        end
        
        function CommStart(obj)
            obj.serialObj = serialPort(obj.portName, obj.baudRate);
            obj.CommOpen = true;
        end
        
        function CommStop(obj)
            instrreset;
            obj.serialObj = nan;
            obj.CommOpen = false;
        end
        
        function out = IsCommOpen(obj)
            out = obj.CommOpen;
        end
        
        function numOfBytes = CommBytesAvailable(obj)
            numOfBytes = obj.serialObj.NumBytesAvailable;
        end
        
        function CommRead(obj)
            while obj.serialObj.NumBytesAvailable < obj.rxPacketLen
                if ~obj.CommOpen
                    break
                end
            end
            if obj.CommOpen
                obj.rawBytes = read(obj.serialObj,obj.rxPacketLen,'uint8');
                % Total Time
                obj.frameData(1) = typecast(uint8(obj.rawBytes(5:8)),'uint32');
                % XYZ Global Forces
                obj.frameData(2) = double(typecast(uint8(obj.rawBytes(9:12)),'int32'));
                obj.frameData(3) = double(typecast(uint8(obj.rawBytes(13:16)),'int32'));
                obj.frameData(4) = double(typecast(uint8(obj.rawBytes(17:20)),'int32'));
                % XYZ Bot Goal
                obj.frameData(5) = double(typecast(uint8(obj.rawBytes(21:24)),'int32'));
                obj.frameData(6) = double(typecast(uint8(obj.rawBytes(25:28)),'int32'));
                obj.frameData(7) = double(typecast(uint8(obj.rawBytes(29:32)),'int32'));
                % XYZ Dot Bot Goal
                obj.frameData(8) = double(typecast(uint8(obj.rawBytes(33:36)),'int32'));
                obj.frameData(9) = double(typecast(uint8(obj.rawBytes(37:40)),'int32'));
                obj.frameData(10) = double(typecast(uint8(obj.rawBytes(41:44)),'int32'));
                % Pres Q
                obj.frameData(11) = double(typecast(uint8(obj.rawBytes(45:48)),'int32'));
                obj.frameData(12) = double(typecast(uint8(obj.rawBytes(49:52)),'int32'));
                obj.frameData(13) = double(typecast(uint8(obj.rawBytes(53:56)),'int32'));
                % Goal Q
                obj.frameData(14) = double(typecast(uint8(obj.rawBytes(57:60)),'int32'));
                obj.frameData(15) = double(typecast(uint8(obj.rawBytes(61:64)),'int32'));
                obj.frameData(16) = double(typecast(uint8(obj.rawBytes(65:68)),'int32'));
                % MassXY and Z
                obj.frameData(17) = double(typecast(uint8(obj.rawBytes(69:72)),'int32'));
                obj.frameData(18) = double(typecast(uint8(obj.rawBytes(73:76)),'int32'));
                % Damping XY and Z
                obj.frameData(19) = double(typecast(uint8(obj.rawBytes(77:80)),'int32'));
                obj.frameData(20) = double(typecast(uint8(obj.rawBytes(81:84)),'int32'));
                % Spring Force
                obj.frameData(21) = double(typecast(uint8(obj.rawBytes(85:88)),'int32'));
                % Other data
                obj.frameData(22) = double(typecast(uint8(obj.rawBytes(89:92)),'int32'));
                % Loop Time
                obj.frameData(23) = typecast(uint8(obj.rawBytes(93:96)),'uint32');
                
                obj.frameData(1) = obj.frameData(1)*0.001;
                obj.frameData(23) = obj.frameData(23)*0.001;
                obj.frameData(2:22) = obj.frameData(2:22)./10000;
            end
        end
        
        function CommSendMassXY(obj, MassXY)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(1);
            writePacket(6:9) = typecast(int32(MassXY*10000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function CommSendMassZ(obj, MassZ)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(2);
            writePacket(10:13) = typecast(int32(MassZ*10000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function CommSendDampingXY(obj, DampingXY)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(4);
            writePacket(14:17) = typecast(int32(DampingXY*10000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function CommSendDampingZ(obj, DampingZ)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(8);
            writePacket(18:21) = typecast(int32(DampingZ*10000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function CommSendSpring(obj, SpringRatio)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(16);
            writePacket(22:25) = typecast(int32(SpringRatio*10000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function CommSendExtForces(obj, eFx, eFy, eFz)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(224);
            writePacket(26:29) = typecast(int32(eFx*10000),'uint8');
            writePacket(30:33) = typecast(int32(eFy*10000),'uint8');
            writePacket(34:37) = typecast(int32(eFz*10000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
    end
    
end