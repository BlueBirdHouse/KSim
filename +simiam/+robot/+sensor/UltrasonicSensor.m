classdef UltrasonicSensor < handle
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    
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
        
        %传感器测量得到的真实数据
        SensorData
                
        %经过匹配以后得到的有关障碍物的数据
        ObstacleData_s = simiam.robot.sensor.data.ObstacleData.empty(0,0);
    end

    
    methods
        function obj = UltrasonicSensor(Sensor_Number)
            %根据US传感器编码设定传感器位置
            obj.location = (obj.location_List(Sensor_Number,:))';
            obj.type = strcat('KheperaIII_US_',num2str(Sensor_Number));
            %测量范围从20cm到4m
            obj.min_range = 20*1e-2;
            obj.max_range = 4;
            
            %为这个传感器关联一个噪声模型
            obj.noise_model = simiam.robot.sensor.noise.Khepera3_US_Noise(Sensor_Number);
            
            %为这个传感器关联一个数据模型
            obj.SensorData = simiam.robot.sensor.data.SensorData(Sensor_Number);
            
        end
        
        function ObstacleData_Cleaner(obj)
            %清除所有ObstacleData_s当中的数据
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
        %下面是观测方程：
        function r_s = r_s_Plane(obj,P_r,P_theta, x,y,theta, Sensor_location)
            %P_r,P_theta 墙在地图当中的位置
            %x,y,theta 机器人在世界坐标系当中的位置
            %Sensor_location 传感器在机器人坐标上的位置
            x_s_ = Sensor_location(1,1);
            y_s_ = Sensor_location(2,1);
            theta_s_ = Sensor_location(3,1);
            
            r_s = P_r - (sin(theta)*x_s_ + cos(theta)*y_s_ + y)*sin(P_theta) - (cos(theta)*x_s_ - sin(theta)*y_s_ + x)*cos(P_theta);
        end
        
        %观测方程的导数
        function det_r_s = det_r_s_Plane(obj,P_r,P_theta, x,y,theta, Sensor_location)
            %P_r,P_theta 墙在地图当中的位置
            %x,y,theta 机器人在世界坐标系当中的位置
            %Sensor_location 传感器在机器人坐标上的位置
            x_s_ = Sensor_location(1,1);
            y_s_ = Sensor_location(2,1);
            theta_s_ = Sensor_location(3,1);
            
            det_r_x = -cos(P_theta);
            det_r_y = -sin(P_theta);
            det_r_theta = x_s_ * sin(theta - P_theta) + y_s_ * cos(theta - P_theta);%这个地方我确实认为应该是+
            %det_r_theta = x_s_ * sin(theta - P_theta) - y_s_ * cos(theta - P_theta);
            
            
            det_r_s = [det_r_x det_r_y det_r_theta];
        end
        
        function r_s = r_s_Corner(obj,P_x,P_y, x,y,theta, Sensor_location)
            %P_x,P_y 拐角在地图当中的位置
            %x,y,theta 机器人在世界坐标系当中的位置
            %Sensor_location 传感器在机器人坐标上的位置
            x_s_ = Sensor_location(1,1);
            y_s_ = Sensor_location(2,1);
            theta_s_ = Sensor_location(3,1);
            
            r_s = sqrt((P_x - (cos(theta)*x_s_ - sin(theta)*y_s_ + x))^2 + (P_y - (sin(theta)*x_s_ + cos(theta)*y_s_ + y))^2);
        end
        
        %观测方程的导数
        function det_r_s = det_r_s_Corner(obj,P_x,P_y, x,y,theta, Sensor_location)
            %P_x,P_y 拐角在地图当中的位置
            %x,y,theta 机器人在世界坐标系当中的位置
            %Sensor_location 传感器在机器人坐标上的位置
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

