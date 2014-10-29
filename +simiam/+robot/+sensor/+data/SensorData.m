classdef SensorData < handle
    %UNTITLED2 此处显示有关此类的摘要
    %   传感器数据，进过EKF匹配以后就进入ObstacleData
    
    properties
        %真实传感器数据列表
        RealDataList
        
        %匹配成功的数据
        MatchedData_List
        %MatchedDataType_List
        
        %该传感器数据的来源
        Sensor_Number
    end
    
    methods
        function obj = SensorData(Sensor_Number)
            obj.Sensor_Number = Sensor_Number;
        end
    end
    
end

