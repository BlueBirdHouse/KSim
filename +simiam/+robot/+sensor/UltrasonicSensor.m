classdef UltrasonicSensor < handle
    %UNTITLED �˴���ʾ�йش����ժҪ
    %   �˴���ʾ��ϸ˵��
    
     properties (Constant = true)
         location_List = [0.005 0.058 deg2rad(90);
            0.04 0.0425 deg2rad(45);
            0.053 0 deg2rad(0);
            0.04 -0.0425 deg2rad(-45);
            0.005 -0.058 deg2rad(-90)]
     end
    
    properties
        type
        
        min_range   % minimum range of US sensor
        max_range   % maximum range of US sensor
        spread      % view angle of US sensor
        
        location % placement US on robot
        
        noise_model
        
        %�����������õ�����ʵ����
        SensorData
                
        %����ƥ���Ժ�õ����й��ϰ��������
        ObstacleData_s = simiam.robot.sensor.data.ObstacleData.empty(0,0);
    end

    
    methods
        function obj = UltrasonicSensor(Sensor_Number)
            %����US�����������趨������λ��
            obj.location = (obj.location_List(Sensor_Number,:))';
            obj.type = strcat('KheperaIII_US_',num2str(Sensor_Number));
            %������Χ��20cm��4m
            obj.min_range = 20*1e-2;
            obj.max_range = 4;
            
            %Ϊ�������������һ������ģ��
            obj.noise_model = simiam.robot.sensor.noise.Khepera3_US_Noise(Sensor_Number);
            
            %Ϊ�������������һ������ģ��
            obj.SensorData = simiam.robot.sensor.data.SensorData(Sensor_Number);
            
        end
        
        function ObstacleData_Cleaner(obj)
            %�������ObstacleData_s���е�����
            obj.ObstacleData_s = simiam.robot.sensor.data.ObstacleData.empty(1,0);
        end
    end
    
    methods (Static)
        function rad = deg2rad(deg)
            rad = deg*pi/180;
        end

        function deg = rad2deg(rad)
            deg = rad*180/pi;
        end
    end
    
   methods     
        %�����ǹ۲ⷽ�̣�
        function r_s = r_s_Plane(obj,P_r,P_theta, x,y,theta, Sensor_location)
            %P_r,P_theta ǽ�ڵ�ͼ���е�λ��
            %x,y,theta ����������������ϵ���е�λ��
            %Sensor_location �������ڻ����������ϵ�λ��
            x_s_ = Sensor_location(1,1);
            y_s_ = Sensor_location(2,1);
            theta_s_ = Sensor_location(3,1);
            
            r_s = P_r - (sin(theta)*x_s_ + cos(theta)*y_s_ + y)*sin(P_theta) - (cos(theta)*x_s_ - sin(theta)*y_s_ + x)*cos(P_theta);
        end
        
        %�۲ⷽ�̵ĵ���
        function det_r_s = det_r_s_Plane(obj,P_r,P_theta, x,y,theta, Sensor_location)
            %P_r,P_theta ǽ�ڵ�ͼ���е�λ��
            %x,y,theta ����������������ϵ���е�λ��
            %Sensor_location �������ڻ����������ϵ�λ��
            x_s_ = Sensor_location(1,1);
            y_s_ = Sensor_location(2,1);
            theta_s_ = Sensor_location(3,1);
            
            det_r_x = -cos(P_theta);
            det_r_y = -sin(P_theta);
            det_r_theta = x_s_ * sin(theta - P_theta) + y_s_ * cos(theta - P_theta);%����ط���ȷʵ��ΪӦ����+
            %det_r_theta = x_s_ * sin(theta - P_theta) - y_s_ * cos(theta - P_theta);
            
            
            det_r_s = [det_r_x det_r_y det_r_theta];
        end
        
        function r_s = r_s_Corner(obj,P_x,P_y, x,y,theta, Sensor_location)
            %P_x,P_y �ս��ڵ�ͼ���е�λ��
            %x,y,theta ����������������ϵ���е�λ��
            %Sensor_location �������ڻ����������ϵ�λ��
            x_s_ = Sensor_location(1,1);
            y_s_ = Sensor_location(2,1);
            theta_s_ = Sensor_location(3,1);
            
            r_s = sqrt((P_x - (cos(theta)*x_s_ - sin(theta)*y_s_ + x))^2 + (P_y - (sin(theta)*x_s_ + cos(theta)*y_s_ + y))^2);
        end
        
        %�۲ⷽ�̵ĵ���
        function det_r_s = det_r_s_Corner(obj,P_x,P_y, x,y,theta, Sensor_location)
            %P_x,P_y �ս��ڵ�ͼ���е�λ��
            %x,y,theta ����������������ϵ���е�λ��
            %Sensor_location �������ڻ����������ϵ�λ��
            x_s_ = Sensor_location(1,1);
            y_s_ = Sensor_location(2,1);
            theta_s_ = Sensor_location(3,1);
            
            d = sqrt((P_x - (cos(theta)*x_s_ - sin(theta)*y_s_ + x))^2 + (P_y - (sin(theta)*x_s_ + cos(theta)*y_s_ + y))^2);
            
            det_r_x = x + x_s_*cos(theta) - y_s_*sin(theta) - P_x;
            det_r_y = y + x_s_*sin(theta) + y_s_*cos(theta) - P_y;
            det_r_theta = 0;
            
            det_r_s = (1/d)*[det_r_x det_r_y det_r_theta];
        end
        
        
        
    end
    
end

