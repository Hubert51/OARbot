classdef KinovaPose < robotics.ros.Message
    %KinovaPose MATLAB implementation of kinova_msgs/KinovaPose
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'kinova_msgs/KinovaPose' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'e831d993faea563f6fe69d7db9b384c9' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        X
        Y
        Z
        ThetaX
        ThetaY
        ThetaZ
    end
    
    properties (Constant, Hidden)
        PropertyList = {'ThetaX', 'ThetaY', 'ThetaZ', 'X', 'Y', 'Z'} % List of non-constant message properties
        ROSPropertyList = {'ThetaX', 'ThetaY', 'ThetaZ', 'X', 'Y', 'Z'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = KinovaPose(msg)
            %KinovaPose Construct the message object KinovaPose
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function x = get.X(obj)
            %get.X Get the value for property X
            x = single(obj.JavaMessage.getX);
        end
        
        function set.X(obj, x)
            %set.X Set the value for property X
            validateattributes(x, {'numeric'}, {'nonempty', 'scalar'}, 'KinovaPose', 'X');
            
            obj.JavaMessage.setX(x);
        end
        
        function y = get.Y(obj)
            %get.Y Get the value for property Y
            y = single(obj.JavaMessage.getY);
        end
        
        function set.Y(obj, y)
            %set.Y Set the value for property Y
            validateattributes(y, {'numeric'}, {'nonempty', 'scalar'}, 'KinovaPose', 'Y');
            
            obj.JavaMessage.setY(y);
        end
        
        function z = get.Z(obj)
            %get.Z Get the value for property Z
            z = single(obj.JavaMessage.getZ);
        end
        
        function set.Z(obj, z)
            %set.Z Set the value for property Z
            validateattributes(z, {'numeric'}, {'nonempty', 'scalar'}, 'KinovaPose', 'Z');
            
            obj.JavaMessage.setZ(z);
        end
        
        function thetax = get.ThetaX(obj)
            %get.ThetaX Get the value for property ThetaX
            thetax = single(obj.JavaMessage.getThetaX);
        end
        
        function set.ThetaX(obj, thetax)
            %set.ThetaX Set the value for property ThetaX
            validateattributes(thetax, {'numeric'}, {'nonempty', 'scalar'}, 'KinovaPose', 'ThetaX');
            
            obj.JavaMessage.setThetaX(thetax);
        end
        
        function thetay = get.ThetaY(obj)
            %get.ThetaY Get the value for property ThetaY
            thetay = single(obj.JavaMessage.getThetaY);
        end
        
        function set.ThetaY(obj, thetay)
            %set.ThetaY Set the value for property ThetaY
            validateattributes(thetay, {'numeric'}, {'nonempty', 'scalar'}, 'KinovaPose', 'ThetaY');
            
            obj.JavaMessage.setThetaY(thetay);
        end
        
        function thetaz = get.ThetaZ(obj)
            %get.ThetaZ Get the value for property ThetaZ
            thetaz = single(obj.JavaMessage.getThetaZ);
        end
        
        function set.ThetaZ(obj, thetaz)
            %set.ThetaZ Set the value for property ThetaZ
            validateattributes(thetaz, {'numeric'}, {'nonempty', 'scalar'}, 'KinovaPose', 'ThetaZ');
            
            obj.JavaMessage.setThetaZ(thetaz);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.X = obj.X;
            cpObj.Y = obj.Y;
            cpObj.Z = obj.Z;
            cpObj.ThetaX = obj.ThetaX;
            cpObj.ThetaY = obj.ThetaY;
            cpObj.ThetaZ = obj.ThetaZ;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.X = strObj.X;
            obj.Y = strObj.Y;
            obj.Z = strObj.Z;
            obj.ThetaX = strObj.ThetaX;
            obj.ThetaY = strObj.ThetaY;
            obj.ThetaZ = strObj.ThetaZ;
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.X = obj.X;
            strObj.Y = obj.Y;
            strObj.Z = obj.Z;
            strObj.ThetaX = obj.ThetaX;
            strObj.ThetaY = obj.ThetaY;
            strObj.ThetaZ = obj.ThetaZ;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.kinova_msgs.KinovaPose.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.kinova_msgs.KinovaPose;
            obj.reload(strObj);
        end
    end
end