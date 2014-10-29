classdef ObstacleData < handle
    %UNTITLED2 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties 
        Type
        Visible
        Map_Number %地图上的第几号障碍物
        Matched
        
        %伪观测信息
        Pseudo_observation
        %观测方程的导数带入预测信息以后的值
        Diff_Pseudo_observation
        
        %为了模拟而产生的被噪声污染的观测数据，实际使用的时候应该填充SensorData数据库
        Pseudo_observationWithNoise
        
        %新息数据，再次强调，如果是真实操作，信息数据一定是SensorData内的数据与Pseudo_observationWithNoise的插值
        %由于一次测量声波可以产生多次回波，所以新息的维数是不一样的
        Innovation
        %Innovation_Variance
        
        %匹配成功的数据
        Matched_SensorData 
        Matched_PredictionData
        
        %这个是匹配成功以后的数据均值和方差
        MatchedData_Mean
        MatchedData_Variance
        %匹配数据
        MatchedData
        
        %该障碍物由哪个传感器测得
        Sensor_Number
    end
    
    methods
        function obj = ObstacleData(Sensor_Number)
            obj.Sensor_Number = Sensor_Number;
        end
    end
    
end

