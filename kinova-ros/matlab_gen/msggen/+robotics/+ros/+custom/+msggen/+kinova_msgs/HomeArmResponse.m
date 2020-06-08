classdef HomeArmResponse < robotics.ros.Message
    %HomeArmResponse MATLAB implementation of kinova_msgs/HomeArmResponse
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'kinova_msgs/HomeArmResponse' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '46e470f2c1a7177398c57a43eafe8d67' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        HomearmResult
    end
    
    properties (Constant, Hidden)
        PropertyList = {'HomearmResult'} % List of non-constant message properties
        ROSPropertyList = {'homearm_result'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = HomeArmResponse(msg)
            %HomeArmResponse Construct the message object HomeArmResponse
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
        
        function homearmresult = get.HomearmResult(obj)
            %get.HomearmResult Get the value for property HomearmResult
            homearmresult = char(obj.JavaMessage.getHomearmResult);
        end
        
        function set.HomearmResult(obj, homearmresult)
            %set.HomearmResult Set the value for property HomearmResult
            homearmresult = convertStringsToChars(homearmresult);
            
            validateattributes(homearmresult, {'char', 'string'}, {}, 'HomeArmResponse', 'HomearmResult');
            
            obj.JavaMessage.setHomearmResult(homearmresult);
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
            cpObj.HomearmResult = obj.HomearmResult;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.HomearmResult = strObj.HomearmResult;
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
            
            strObj.HomearmResult = obj.HomearmResult;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.kinova_msgs.HomeArmResponse.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.kinova_msgs.HomeArmResponse;
            obj.reload(strObj);
        end
    end
end