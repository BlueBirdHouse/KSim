classdef SensorData < handle
    %UNTITLED2 �˴���ʾ�йش����ժҪ
    %   ���������ݣ�����EKFƥ���Ժ�ͽ���ObstacleData
    
    properties
        %��ʵ�����������б�
        RealDataList
        
        %ƥ��ɹ�������
        MatchedData_List
        %MatchedDataType_List
        
        %�ô��������ݵ���Դ
        Sensor_Number
    end
    
    methods
        function obj = SensorData(Sensor_Number)
            obj.Sensor_Number = Sensor_Number;
        end
    end
    
end

