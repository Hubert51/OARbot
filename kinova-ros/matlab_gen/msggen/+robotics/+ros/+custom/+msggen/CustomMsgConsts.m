classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    properties (Constant)
        kinova_msgs_AddPoseToCartesianTrajectory = 'kinova_msgs/AddPoseToCartesianTrajectory'
        kinova_msgs_AddPoseToCartesianTrajectoryRequest = 'kinova_msgs/AddPoseToCartesianTrajectoryRequest'
        kinova_msgs_AddPoseToCartesianTrajectoryResponse = 'kinova_msgs/AddPoseToCartesianTrajectoryResponse'
        kinova_msgs_ArmJointAnglesAction = 'kinova_msgs/ArmJointAnglesAction'
        kinova_msgs_ArmJointAnglesActionFeedback = 'kinova_msgs/ArmJointAnglesActionFeedback'
        kinova_msgs_ArmJointAnglesActionGoal = 'kinova_msgs/ArmJointAnglesActionGoal'
        kinova_msgs_ArmJointAnglesActionResult = 'kinova_msgs/ArmJointAnglesActionResult'
        kinova_msgs_ArmJointAnglesFeedback = 'kinova_msgs/ArmJointAnglesFeedback'
        kinova_msgs_ArmJointAnglesGoal = 'kinova_msgs/ArmJointAnglesGoal'
        kinova_msgs_ArmJointAnglesResult = 'kinova_msgs/ArmJointAnglesResult'
        kinova_msgs_ArmPoseAction = 'kinova_msgs/ArmPoseAction'
        kinova_msgs_ArmPoseActionFeedback = 'kinova_msgs/ArmPoseActionFeedback'
        kinova_msgs_ArmPoseActionGoal = 'kinova_msgs/ArmPoseActionGoal'
        kinova_msgs_ArmPoseActionResult = 'kinova_msgs/ArmPoseActionResult'
        kinova_msgs_ArmPoseFeedback = 'kinova_msgs/ArmPoseFeedback'
        kinova_msgs_ArmPoseGoal = 'kinova_msgs/ArmPoseGoal'
        kinova_msgs_ArmPoseResult = 'kinova_msgs/ArmPoseResult'
        kinova_msgs_Arm_KinovaPoseAction = 'kinova_msgs/Arm_KinovaPoseAction'
        kinova_msgs_Arm_KinovaPoseActionFeedback = 'kinova_msgs/Arm_KinovaPoseActionFeedback'
        kinova_msgs_Arm_KinovaPoseActionGoal = 'kinova_msgs/Arm_KinovaPoseActionGoal'
        kinova_msgs_Arm_KinovaPoseActionResult = 'kinova_msgs/Arm_KinovaPoseActionResult'
        kinova_msgs_Arm_KinovaPoseFeedback = 'kinova_msgs/Arm_KinovaPoseFeedback'
        kinova_msgs_Arm_KinovaPoseGoal = 'kinova_msgs/Arm_KinovaPoseGoal'
        kinova_msgs_Arm_KinovaPoseResult = 'kinova_msgs/Arm_KinovaPoseResult'
        kinova_msgs_CartesianForce = 'kinova_msgs/CartesianForce'
        kinova_msgs_ClearTrajectories = 'kinova_msgs/ClearTrajectories'
        kinova_msgs_ClearTrajectoriesRequest = 'kinova_msgs/ClearTrajectoriesRequest'
        kinova_msgs_ClearTrajectoriesResponse = 'kinova_msgs/ClearTrajectoriesResponse'
        kinova_msgs_FingerPosition = 'kinova_msgs/FingerPosition'
        kinova_msgs_HomeArm = 'kinova_msgs/HomeArm'
        kinova_msgs_HomeArmRequest = 'kinova_msgs/HomeArmRequest'
        kinova_msgs_HomeArmResponse = 'kinova_msgs/HomeArmResponse'
        kinova_msgs_JointAngles = 'kinova_msgs/JointAngles'
        kinova_msgs_JointTorque = 'kinova_msgs/JointTorque'
        kinova_msgs_JointVelocity = 'kinova_msgs/JointVelocity'
        kinova_msgs_KinovaPose = 'kinova_msgs/KinovaPose'
        kinova_msgs_PoseVelocity = 'kinova_msgs/PoseVelocity'
        kinova_msgs_RunCOMParametersEstimation = 'kinova_msgs/RunCOMParametersEstimation'
        kinova_msgs_RunCOMParametersEstimationRequest = 'kinova_msgs/RunCOMParametersEstimationRequest'
        kinova_msgs_RunCOMParametersEstimationResponse = 'kinova_msgs/RunCOMParametersEstimationResponse'
        kinova_msgs_SetEndEffectorOffset = 'kinova_msgs/SetEndEffectorOffset'
        kinova_msgs_SetEndEffectorOffsetRequest = 'kinova_msgs/SetEndEffectorOffsetRequest'
        kinova_msgs_SetEndEffectorOffsetResponse = 'kinova_msgs/SetEndEffectorOffsetResponse'
        kinova_msgs_SetFingersPositionAction = 'kinova_msgs/SetFingersPositionAction'
        kinova_msgs_SetFingersPositionActionFeedback = 'kinova_msgs/SetFingersPositionActionFeedback'
        kinova_msgs_SetFingersPositionActionGoal = 'kinova_msgs/SetFingersPositionActionGoal'
        kinova_msgs_SetFingersPositionActionResult = 'kinova_msgs/SetFingersPositionActionResult'
        kinova_msgs_SetFingersPositionFeedback = 'kinova_msgs/SetFingersPositionFeedback'
        kinova_msgs_SetFingersPositionGoal = 'kinova_msgs/SetFingersPositionGoal'
        kinova_msgs_SetFingersPositionResult = 'kinova_msgs/SetFingersPositionResult'
        kinova_msgs_SetForceControlParams = 'kinova_msgs/SetForceControlParams'
        kinova_msgs_SetForceControlParamsRequest = 'kinova_msgs/SetForceControlParamsRequest'
        kinova_msgs_SetForceControlParamsResponse = 'kinova_msgs/SetForceControlParamsResponse'
        kinova_msgs_SetNullSpaceModeState = 'kinova_msgs/SetNullSpaceModeState'
        kinova_msgs_SetNullSpaceModeStateRequest = 'kinova_msgs/SetNullSpaceModeStateRequest'
        kinova_msgs_SetNullSpaceModeStateResponse = 'kinova_msgs/SetNullSpaceModeStateResponse'
        kinova_msgs_SetTorqueControlMode = 'kinova_msgs/SetTorqueControlMode'
        kinova_msgs_SetTorqueControlModeRequest = 'kinova_msgs/SetTorqueControlModeRequest'
        kinova_msgs_SetTorqueControlModeResponse = 'kinova_msgs/SetTorqueControlModeResponse'
        kinova_msgs_SetTorqueControlParameters = 'kinova_msgs/SetTorqueControlParameters'
        kinova_msgs_SetTorqueControlParametersRequest = 'kinova_msgs/SetTorqueControlParametersRequest'
        kinova_msgs_SetTorqueControlParametersResponse = 'kinova_msgs/SetTorqueControlParametersResponse'
        kinova_msgs_Start = 'kinova_msgs/Start'
        kinova_msgs_StartRequest = 'kinova_msgs/StartRequest'
        kinova_msgs_StartResponse = 'kinova_msgs/StartResponse'
        kinova_msgs_Stop = 'kinova_msgs/Stop'
        kinova_msgs_StopRequest = 'kinova_msgs/StopRequest'
        kinova_msgs_StopResponse = 'kinova_msgs/StopResponse'
        kinova_msgs_ZeroTorques = 'kinova_msgs/ZeroTorques'
        kinova_msgs_ZeroTorquesRequest = 'kinova_msgs/ZeroTorquesRequest'
        kinova_msgs_ZeroTorquesResponse = 'kinova_msgs/ZeroTorquesResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(59, 1);
                msgList{1} = 'kinova_msgs/AddPoseToCartesianTrajectoryRequest';
                msgList{2} = 'kinova_msgs/AddPoseToCartesianTrajectoryResponse';
                msgList{3} = 'kinova_msgs/ArmJointAnglesAction';
                msgList{4} = 'kinova_msgs/ArmJointAnglesActionFeedback';
                msgList{5} = 'kinova_msgs/ArmJointAnglesActionGoal';
                msgList{6} = 'kinova_msgs/ArmJointAnglesActionResult';
                msgList{7} = 'kinova_msgs/ArmJointAnglesFeedback';
                msgList{8} = 'kinova_msgs/ArmJointAnglesGoal';
                msgList{9} = 'kinova_msgs/ArmJointAnglesResult';
                msgList{10} = 'kinova_msgs/ArmPoseAction';
                msgList{11} = 'kinova_msgs/ArmPoseActionFeedback';
                msgList{12} = 'kinova_msgs/ArmPoseActionGoal';
                msgList{13} = 'kinova_msgs/ArmPoseActionResult';
                msgList{14} = 'kinova_msgs/ArmPoseFeedback';
                msgList{15} = 'kinova_msgs/ArmPoseGoal';
                msgList{16} = 'kinova_msgs/ArmPoseResult';
                msgList{17} = 'kinova_msgs/Arm_KinovaPoseAction';
                msgList{18} = 'kinova_msgs/Arm_KinovaPoseActionFeedback';
                msgList{19} = 'kinova_msgs/Arm_KinovaPoseActionGoal';
                msgList{20} = 'kinova_msgs/Arm_KinovaPoseActionResult';
                msgList{21} = 'kinova_msgs/Arm_KinovaPoseFeedback';
                msgList{22} = 'kinova_msgs/Arm_KinovaPoseGoal';
                msgList{23} = 'kinova_msgs/Arm_KinovaPoseResult';
                msgList{24} = 'kinova_msgs/CartesianForce';
                msgList{25} = 'kinova_msgs/ClearTrajectoriesRequest';
                msgList{26} = 'kinova_msgs/ClearTrajectoriesResponse';
                msgList{27} = 'kinova_msgs/FingerPosition';
                msgList{28} = 'kinova_msgs/HomeArmRequest';
                msgList{29} = 'kinova_msgs/HomeArmResponse';
                msgList{30} = 'kinova_msgs/JointAngles';
                msgList{31} = 'kinova_msgs/JointTorque';
                msgList{32} = 'kinova_msgs/JointVelocity';
                msgList{33} = 'kinova_msgs/KinovaPose';
                msgList{34} = 'kinova_msgs/PoseVelocity';
                msgList{35} = 'kinova_msgs/RunCOMParametersEstimationRequest';
                msgList{36} = 'kinova_msgs/RunCOMParametersEstimationResponse';
                msgList{37} = 'kinova_msgs/SetEndEffectorOffsetRequest';
                msgList{38} = 'kinova_msgs/SetEndEffectorOffsetResponse';
                msgList{39} = 'kinova_msgs/SetFingersPositionAction';
                msgList{40} = 'kinova_msgs/SetFingersPositionActionFeedback';
                msgList{41} = 'kinova_msgs/SetFingersPositionActionGoal';
                msgList{42} = 'kinova_msgs/SetFingersPositionActionResult';
                msgList{43} = 'kinova_msgs/SetFingersPositionFeedback';
                msgList{44} = 'kinova_msgs/SetFingersPositionGoal';
                msgList{45} = 'kinova_msgs/SetFingersPositionResult';
                msgList{46} = 'kinova_msgs/SetForceControlParamsRequest';
                msgList{47} = 'kinova_msgs/SetForceControlParamsResponse';
                msgList{48} = 'kinova_msgs/SetNullSpaceModeStateRequest';
                msgList{49} = 'kinova_msgs/SetNullSpaceModeStateResponse';
                msgList{50} = 'kinova_msgs/SetTorqueControlModeRequest';
                msgList{51} = 'kinova_msgs/SetTorqueControlModeResponse';
                msgList{52} = 'kinova_msgs/SetTorqueControlParametersRequest';
                msgList{53} = 'kinova_msgs/SetTorqueControlParametersResponse';
                msgList{54} = 'kinova_msgs/StartRequest';
                msgList{55} = 'kinova_msgs/StartResponse';
                msgList{56} = 'kinova_msgs/StopRequest';
                msgList{57} = 'kinova_msgs/StopResponse';
                msgList{58} = 'kinova_msgs/ZeroTorquesRequest';
                msgList{59} = 'kinova_msgs/ZeroTorquesResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(12, 1);
                svcList{1} = 'kinova_msgs/AddPoseToCartesianTrajectory';
                svcList{2} = 'kinova_msgs/ClearTrajectories';
                svcList{3} = 'kinova_msgs/HomeArm';
                svcList{4} = 'kinova_msgs/RunCOMParametersEstimation';
                svcList{5} = 'kinova_msgs/SetEndEffectorOffset';
                svcList{6} = 'kinova_msgs/SetForceControlParams';
                svcList{7} = 'kinova_msgs/SetNullSpaceModeState';
                svcList{8} = 'kinova_msgs/SetTorqueControlMode';
                svcList{9} = 'kinova_msgs/SetTorqueControlParameters';
                svcList{10} = 'kinova_msgs/Start';
                svcList{11} = 'kinova_msgs/Stop';
                svcList{12} = 'kinova_msgs/ZeroTorques';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(4, 1);
                actList{1} = 'kinova_msgs/ArmJointAngles';
                actList{2} = 'kinova_msgs/ArmPose';
                actList{3} = 'kinova_msgs/Arm_KinovaPose';
                actList{4} = 'kinova_msgs/SetFingersPosition';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
