classdef ObstacleData < handle
    %UNTITLED2 �˴���ʾ�йش����ժҪ
    %   �˴���ʾ��ϸ˵��
    
    properties 
        Type
        Visible
        Map_Number %��ͼ�ϵĵڼ����ϰ���
        Matched
        
        %α�۲���Ϣ
        Pseudo_observation
        %�۲ⷽ�̵ĵ�������Ԥ����Ϣ�Ժ��ֵ
        Diff_Pseudo_observation
        
        %Ϊ��ģ��������ı�������Ⱦ�Ĺ۲����ݣ�ʵ��ʹ�õ�ʱ��Ӧ�����SensorData���ݿ�
        Pseudo_observationWithNoise
        
        %��Ϣ���ݣ��ٴ�ǿ�����������ʵ��������Ϣ����һ����SensorData�ڵ�������Pseudo_observationWithNoise�Ĳ�ֵ
        %����һ�β����������Բ�����λز���������Ϣ��ά���ǲ�һ����
        Innovation
        %Innovation_Variance
        
        %ƥ��ɹ�������
        Matched_SensorData 
        Matched_PredictionData
        
        %�����ƥ��ɹ��Ժ�����ݾ�ֵ�ͷ���
        MatchedData_Mean
        MatchedData_Variance
        %ƥ������
        MatchedData
        
        %���ϰ������ĸ����������
        Sensor_Number
    end
    
    methods
        function obj = ObstacleData(Sensor_Number)
            obj.Sensor_Number = Sensor_Number;
        end
    end
    
end

