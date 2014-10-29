classdef EKF < handle
    %UNTITLED �˴���ʾ�йش����ժҪ
    %   ע�⣬�˲���ֻ�ܹ�����ʼ��һ�Σ�
    
     properties (Constant = true)
        %ϵͳ������
        Q = diag([(5e-3)^2 (5e-3)^2 (deg2rad(0.4))^2]); %5cm �� 4��
        
        %��ʼ����ͼ
        Map = simiam.robot.map.Map_Rectangle('Khepera3');
        
     end
    
    properties
        type %�����ͺŵĻ�������ʹ���˲���
        
        Pose %�����˵�λ�˹���
        Diff_Pose %ϵͳ���̵ĵ���
        Pose_k_1 %�����˵�λ��Ԥ��
        
        Variance %�����Ԥ�ⷽ��
        Variance_k_1
        
        IF_Observation_Success %��ʾ�۲⼰��Ϣ�Ĳ����Ƿ�ɹ�
        IF_Filter_Success %��һ�ε��˲�����Ƿ�ɹ�
        MatchedDataNumber = 0; %��ʾƥ���ϵ���Ч���ݸ���
        
        %�����˽ṹ����
        r_R
        r_L
        b
        
        %�����ź�U
        u 

        %��ʼ��������
        US = simiam.robot.sensor.UltrasonicSensor.empty(1,0);
        
        %��������ʾ������4*2��ͼƬ��
        Obstacle_Plane_TextBox_1
        Obstacle_Plane_TextBox_2
        Obstacle_Plane_TextBox_3
        Obstacle_Plane_TextBox_4
        
        Obstacle_Point_TextBox_1
        Obstacle_Point_TextBox_2
        Obstacle_Point_TextBox_3
        Obstacle_Point_TextBox_4
        
        
        %��ʾ�Ƿ���Ҫ��ʾ�۲����
        User_ShowObject
    end

    methods (Static)
        function rad = deg2rad(deg)
            rad = deg*pi/180;
        end

        function deg = rad2deg(rad)
            deg = rad*180/pi;
        end

        function Rad_Fixed = FixAngle(Rad)
            %������������ĽǶȣ��������ǣ�-pi-pi
            while (Rad > pi) 
                Rad = Rad - 2 * pi;
            end
            while (Rad < -pi) 
                Rad = Rad + 2 * pi;
            end  
            Rad_Fixed = Rad;
        end
        
        function R = get_transformation_matrix(x, y, theta)
            R = [cos(theta) -sin(theta) x; sin(theta) cos(theta) y; 0 0 1];
        end
        
    end
    
    methods
        function obj = EKF(type,pose,Variance,wheel_radius,wheel_base_length)
            obj.type = type;
            if(strcmp(obj.type,'Khepera3'))
                %�����K3�����ˣ�������K3��صĳ�ʼ������
                %ע�⣬���������ѭ������ֻ�ܹ�������һ��
                obj.IF_Filter_Success = false;
                
                %��ʼ��λ�˺�λ�˵�Ԥ��
                obj.Pose = pose;
                obj.Pose_k_1 = pose;
                
                %��ʼ��λ�˷����λ�˷����Ԥ��
                obj.Variance = Variance;
                obj.Variance_k_1 = Variance;
                
                %��ʼ���ṹ����
                obj.r_R = wheel_radius;
                obj.r_L = wheel_radius;
                obj.b = wheel_base_length;
                
                %��ʼ��������
                for i = 1:5
                    obj.US(i) = simiam.robot.sensor.UltrasonicSensor(i);
                end
                  
                %�ڼ��Ӵ����ϳ�ʼ�������ı���������ʾ������״̬
                obj.Obstacle_Plane_TextBox_1 = annotation('textbox',...
                    [0.8 0.6 0.05 0.05],...
                    'String',{'ƽ���ϰ�1��','������  ƥ��','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Plane_TextBox_2 = annotation('textbox',...
                    [0.45 0.23 0.05 0.05],...
                    'String',{'ƽ���ϰ�2��','������  ƥ��','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Plane_TextBox_3 = annotation('textbox',...
                    [0.05 0.6 0.05 0.05],...
                    'String',{'ƽ���ϰ�3��','������  ƥ��','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Plane_TextBox_4 = annotation('textbox',...
                    [0.45 0.89 0.05 0.05],...
                    'String',{'ƽ���ϰ�4��','������  ƥ��','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Point_TextBox_1 = annotation('textbox',...
                    [0.8 0.89 0.05 0.05],...
                    'String',{'��״�ϰ�1��','������  ƥ��','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Point_TextBox_2 = annotation('textbox',...
                    [0.8 0.23 0.05 0.05],...
                    'String',{'��״�ϰ�2��','������  ƥ��','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Point_TextBox_3 = annotation('textbox',...
                    [0.05 0.23 0.05 0.05],...
                    'String',{'��״�ϰ�3��','������  ƥ��','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Point_TextBox_4 = annotation('textbox',...
                    [0.05 0.89 0.05 0.05],...
                    'String',{'��״�ϰ�4��','������  ƥ��','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                Temp = 0;
            end
        end
        
        function Prediction_Pose_EKF(obj,u)
            %ִ���˲���Ԥ��
            obj.u = u;
            
            T_k = u(1,1);
            Det_Theta_k = u(2,1);
            
            x = obj.Pose(1,1);
            y = obj.Pose(2,1);
            theta = obj.Pose(3,1);
            
            Temp = theta + Det_Theta_k;
            Temp = obj.FixAngle(Temp);
            
            x_k_1 = x + T_k * cos(Temp);
            y_k_1 = y + T_k * sin(Temp);
            theta_k_1 = Temp;
            
            obj.Pose_k_1 = [x_k_1 y_k_1 theta_k_1]';
            
            obj.Diff_Pose = [1 0 -T_k * sin(theta) ; 0 1 T_k * cos(theta) ; 0 0 1];
        end
        
        function u = Prediction_u_Maker(obj,dt,vel_r,vel_l)
            %�����˲��㷨��Ҫ�������u
            R = (obj.r_R+obj.r_L)/2;
            L = obj.b;
            
            v = R/2*(vel_r+vel_l);
            w = R/L*(vel_r-vel_l);
            
            T_k = v * dt;
            Det_Theta_k = w * dt;
            
            u = [T_k ; Det_Theta_k];
        end
                
        function Prediction_Variance_EKF(obj)
            %�ķ��������ڵ���Prediction_Pose_EKF�Ժ����
            Diff = obj.Diff_Pose;
            P = obj.Variance;
            
            obj.Variance_k_1 = Diff * P * (Diff)' + obj.Q;
        end
        
        %����ĺ�������۲ⲿ��
        function Obstacle_Visible(obj,Realpose)
            %��⴫�����ɼ����ϰ�����Ϣ�������ObstacleData
            %������ĽǶȶ�Ҫʹ��2pi�ƶȣ�
%             x_k_1 = obj.Pose_k_1(1,1);
%             y_k_1 = obj.Pose_k_1(2,1);
%             theta_k_1 = obj.Pose_k_1(3,1);
            
            x_k_1 = Realpose.x;
            y_k_1 = Realpose.y;
            theta_k_1 = Realpose.theta;
            theta_k_1 = obj.FixAngle(theta_k_1);
            
            for i = 1:length(obj.US)
                %������ƽ��Ŀ��Ŀɼ���
                Obstacle_Planes = obj.Map.Obstacle_Planes;
                [m,~] = size(Obstacle_Planes);
                for j = 1:m
                    %ȡ����ͼ�ϰ�������
                    Phi_st = Obstacle_Planes(j,2);
                    Beta_t = obj.Map.Planes_Beta;
                    
                    %��ǰ����������������ϵ�µ�ת��
                    a_st = theta_k_1 + obj.US(i).location(3,1);
                    
                    %�����Ƕ�Ϊ2PI�ƶ�
                    Phi_st = obj.Pi_Pi22PI(Phi_st);
                    Beta_t = obj.Pi_Pi22PI(Beta_t);
                    a_st = obj.Pi_Pi22PI(a_st);
                    
                    %����ĽǶ��жϻ���Angile1��Angile2����0�ȵ�ʱ��������
                    Angile1 = Phi_st - Beta_t/2;
                    Angile2 = Phi_st + Beta_t/2;
                    %Angile2 = obj.Pi_Pi22PI(Phi_st + Beta_t/2);
                    
                    %�ж��Ƿ���ͬ��
                    SameSign = Angile1 * Angile2;
                    
                    Angile1 = obj.Pi_Pi22PI(Angile1);
                    Angile2 = obj.Pi_Pi22PI(Angile2);
                    
                    if(SameSign >= 0)
                        Condition = ((Angile1 <= a_st) && (Angile2 >= a_st));
                    else
                        Condition = ((Angile1 <= a_st) || (Angile2 >= a_st));
                    end
                    
                    if(Condition)
                        %˵������ϰ���ɼ��������йظ��ϰ������Ϣ��ObstacleData
                        Temp = simiam.robot.sensor.data.ObstacleData(i);
                        Temp.Type = 'Plane';
                        Temp.Visible = true;
                        Temp.Map_Number = j;
                        
                        obj.US(i).ObstacleData_s = [obj.US(i).ObstacleData_s ; Temp];
                    end
                end
            end    
            
              %�������Ŀ��Ŀɼ���
             for i = 1:length(obj.US)
                 Obstacle_Points = obj.Map.Obstacle_Point;
                 [m,~] = size(Obstacle_Points);
                 for j = 1:m
                     p_x = Obstacle_Points(j,1);
                     p_y = Obstacle_Points(j,2);
                     
                     %�����������Ļ�����������
                     x_s_ = obj.US(i).location(1,1);
                     y_s_ = obj.US(i).location(2,1);
                     Temp = [x_s_ y_s_ 1]';
                     Temp = obj.get_transformation_matrix(x_k_1,y_k_1,theta_k_1) * Temp;
                     
                     x_s = Temp(1,1);
                     y_s = Temp(2,1);
                     
                     Phi_st = atan2(p_y - y_s, p_x - x_s);
                     
                     Beta_t = obj.Map.Point_Beta;
                     a_st = theta_k_1 + obj.US(i).location(3,1);
                     
                     %�����Ƕ�Ϊ2PI�ƶ�
                     Phi_st = obj.Pi_Pi22PI(Phi_st);
                     Beta_t = obj.Pi_Pi22PI(Beta_t);
                     a_st = obj.Pi_Pi22PI(a_st);
                     
                     %����ĽǶ��жϻ���Angile1��Angile2����0�ȵ�ʱ��������
                     Angile1 = Phi_st - Beta_t/2;
                     Angile2 = Phi_st + Beta_t/2;
                     %Angile2 = obj.Pi_Pi22PI(Phi_st + Beta_t/2);
                    
                     %�ж��Ƿ���ͬ��
                     SameSign = Angile1 * Angile2;
                    
                     Angile1 = obj.Pi_Pi22PI(Angile1);
                     Angile2 = obj.Pi_Pi22PI(Angile2);
                    
                     if(SameSign >= 0)
                         Condition = ((Angile1 <= a_st) && (Angile2 >= a_st));
                     else
                         Condition = ((Angile1 <= a_st) || (Angile2 >= a_st));
                     end
                     
                     if(Condition)
                         %˵������ϰ���ɼ��������йظ��ϰ������Ϣ��ObstacleData
                         Temp = simiam.robot.sensor.data.ObstacleData(i);
                         Temp.Type = 'Point';
                         Temp.Visible = true;
                         Temp.Map_Number = j;
                         obj.US(i).ObstacleData_s = [obj.US(i).ObstacleData_s ; Temp];
                     end                     
                 end
             end
        end
        
        function Pseudo_observation(obj,Realpose)
            %������ObstacleData_s�������α�۲���Ϣ
            for i = 1:length(obj.US)
                %����ȡ����Ҫ�����ObstacleData_s����
                ObstacleData = obj.US(i).ObstacleData_s;
                if isempty(ObstacleData) == 0
                    [m,~] = size(ObstacleData);%����ά��
                    for Counter = 1:m
                        Data = ObstacleData(Counter,:);
                        if Data.Visible
                            if strcmp('Plane',Data.Type) %����ƽ��Ŀ��͵�Ŀ�����ֱ���
                                %��ȡ��Ӧ�ĵ�ͼ��Ϣ
                                Map_Number = Data.Map_Number;
                                P_r = obj.Map.Obstacle_Planes(Map_Number,1);
                                P_theta = obj.Map.Obstacle_Planes(Map_Number,2);
                                
%                                 x = obj.Pose_k_1(1,1);
%                                 y = obj.Pose_k_1(2,1);
%                                 theta = obj.Pose_k_1(3,1);
                                
                                x = Realpose.x;
                                y = Realpose.y;
                                theta = Realpose.theta;
                                theta = obj.FixAngle(theta);
                                
                                Sensor_location = (obj.US(i).location_List(i,:))';
                                
                                Data.Pseudo_observation = obj.US(i).r_s_Plane(P_r,P_theta, x,y,theta, Sensor_location);
                                Data.Diff_Pseudo_observation = obj.US(i).det_r_s_Plane(P_r,P_theta, x,y,theta, Sensor_location);
                            else
                                %��ȡ��Ӧ�ĵ�ͼ��Ϣ
                                Map_Number = Data.Map_Number;
                                P_x = obj.Map.Obstacle_Point(Map_Number,1);
                                P_y = obj.Map.Obstacle_Point(Map_Number,2);
                                
%                                 x = obj.Pose_k_1(1,1);
%                                 y = obj.Pose_k_1(2,1);
%                                 theta = obj.Pose_k_1(3,1);
                                
                                x = Realpose.x;
                                y = Realpose.y;
                                theta = Realpose.theta;
                                theta = obj.FixAngle(theta);
                                
                                Sensor_location = (obj.US(i).location_List(i,:))';
                                
                                Data.Pseudo_observation = obj.US(i).r_s_Corner(P_x,P_y, x,y,theta, Sensor_location);
                                Data.Diff_Pseudo_observation = obj.US(i).det_r_s_Corner(P_x,P_y, x,y,theta, Sensor_location);
                            end
                        end
                        %д������
                        ObstacleData(Counter,:) = Data;
                    end
                end
                %д������
                obj.US(i).ObstacleData_s = ObstacleData;
                
            end
        end
        
        function Guess_Data_MeanVariance(obj)
            %������ObstacleData_s�����²���������ľ�ֵ�ͷ���
            for i = 1:length(obj.US)
                %����ȡ����Ҫ�����ObstacleData_s����
                ObstacleData = obj.US(i).ObstacleData_s;
                if isempty(ObstacleData) == 0
                    [m,~] = size(ObstacleData);%����ά��
                    for Counter = 1:m
                        Data = ObstacleData(Counter,:);
                        if Data.Visible
                            %����ȡ��α�۲���Ϣ
                            Pseudo_observ = Data.Pseudo_observation;
                            Data.Pseudo_observationWithNoise = obj.US(i).noise_model.apply_noise(Pseudo_observ);
                            %��д������Ϣ
                            Data.MatchedData_Mean = obj.US(i).noise_model.Mean_Noise;
                            Data.MatchedData_Variance = obj.US(i).noise_model.variance_Noise;
                            
                        end
                        %д������
                        ObstacleData(Counter,:) = Data;
                    end
                end
                %д������
                obj.US(i).ObstacleData_s = ObstacleData;
            end
        end
        
        function InnovationMaker(obj,IsSimulation)
            if(IsSimulation)
                %���û���κ�һ����Ч�۲�Ļ�����ô��һ���˲�����ʧ�ܵ�
                obj.IF_Observation_Success = false;
                for i = 1:length(obj.US)
                    %����ȡ����Ҫ�����ObstacleData_s����
                    ObstacleData = obj.US(i).ObstacleData_s;
                    if isempty(ObstacleData) == 0
                        [m,~] = size(ObstacleData);%����ά��
                        for Counter = 1:m
                            Data = ObstacleData(Counter,:);
                            if Data.Visible
                                %ֻҪ��һ����Ч�۲⣬��ô��˵����һ���˲��ǳɹ���
                                obj.IF_Observation_Success = true;
                                %����ȡ��α�۲���Ϣ
                                observation_ = Data.Pseudo_observation;
                                %������ģ��״̬�£�һ�β��������ķ���ֻ���յ�һ���ز�����ô����۲��ά���ض�����ͬ��
                                Realobservation = Data.Pseudo_observationWithNoise;
                                %������Ϣ
                                Data.Innovation = Realobservation - observation_;
                            end
                            %д������
                            ObstacleData(Counter,:) = Data;
                        end
                    end
                    %д������
                    obj.US(i).ObstacleData_s = ObstacleData;
                end
            end
        end
        
        function Match(obj,IsSimulation)
            %��ƥ�����
            if(IsSimulation)
                %���û���κ�һ���ɹ�ƥ��Ļ�����ô��һ���˲�����ʧ�ܵ�
                obj.IF_Filter_Success = false;
                for i = 1:length(obj.US)
                    %����ȡ����Ҫ�����ObstacleData_s����
                    ObstacleData = obj.US(i).ObstacleData_s;
                    if isempty(ObstacleData) == 0
                        [m,~] = size(ObstacleData);%����ά��
                        for Counter = 1:m
                            Data = ObstacleData(Counter,:);
                            if Data.Visible
                                %��������ƥ�䣬������ģ�⣬���趼ƥ������
                                Data.Matched = true;
                                %��¼�ж�������ƥ������
                                obj.MatchedDataNumber = obj.MatchedDataNumber + 1;
                                obj.IF_Filter_Success = true;
                            end
                            %д������
                            ObstacleData(Counter,:) = Data;
                        end
                    end
                    %д������
                    obj.US(i).ObstacleData_s = ObstacleData;
                end
            end
        end
        
        function Estimate(obj)
            %���û���κ�һ���ɹ�ƥ��Ļ�����ô��һ���˲�����ʧ�ܵ�
            if(obj.IF_Filter_Success)
                %�����������
                det_h = zeros(obj.MatchedDataNumber,3);%�۲ⷽ�̵ĵ�������
                
                V = zeros(obj.MatchedDataNumber,1);%��Ϣ�ļ���
                R = zeros(obj.MatchedDataNumber,obj.MatchedDataNumber); %�۲���������ļ��ϣ��Խ�
                
                Printer = 1;
                
                for i = 1:length(obj.US)
                %����ȡ����Ҫ�����ObstacleData_s����
                    ObstacleData = obj.US(i).ObstacleData_s;
                    if isempty(ObstacleData) == 0
                        [m,~] = size(ObstacleData);%����ά��
                        for Counter = 1:m
                            Data = ObstacleData(Counter,:);
                            if Data.Matched
                                %����˲�����Ҫ������
                                det_h(Printer,:) = Data.Diff_Pseudo_observation;
                                
                                V(Printer,1) = Data.Innovation;
                                R(Printer,Printer) = Data.MatchedData_Variance;
                                
                                Printer = Printer + 1;
                            end
                        end
                    end
                end
                
                K = obj.Variance_k_1 * (det_h') * inv(det_h*obj.Variance_k_1*(det_h') + R);
                obj.Pose = obj.Pose_k_1 + K * V;
                obj.Variance = obj.Variance_k_1 - K * det_h * obj.Variance_k_1;
                %V
                Temp = 0;
            else
                %�����˲�û�гɹ����������һ�ο�Ԥ�⣩
                obj.Pose = obj.Pose_k_1;
                obj.Variance = obj.Variance_k_1;
            end
            Temp = 0;
        end
        
        function Rad_Fixed = Pi_Pi22PI(obj,Rad)
            %��-pi-pi��ת��Ϊ2PI��
            Rad = obj.FixAngle(Rad);
            if(Rad < 0)
                Rad = 2*pi - abs(Rad);
            end
            Rad_Fixed = Rad;
        end

        function CleanAllObstacleData(obj)
            %һ����������д���������
            for i = 1 : length(obj.US)
                obj.US(i).ObstacleData_Cleaner();
            end
            obj.MatchedDataNumber = 0;
        end
        
        function ShowObject(obj)
            %�����ڵ�ͼ����ʾ��Щ�ϰ��ﱻ��Щ��������⵽�ˣ�
            PlaneText1 = {'ƽ���ϰ�1��','������  ƥ��','                ','                ','                ','                ','                '}';
            PlaneText2 = {'ƽ���ϰ�2��','������  ƥ��','                ','                ','                ','                ','                '}';
            PlaneText3 = {'ƽ���ϰ�3��','������  ƥ��','                ','                ','                ','                ','                '}';
            PlaneText4 = {'ƽ���ϰ�4��','������  ƥ��','                ','                ','                ','                ','                '}';
            
            PointText1 = {'��״�ϰ�1��','������  ƥ��','                ','                ','                ','                ','                '}';
            PointText2 = {'��״�ϰ�2��','������  ƥ��','                ','                ','                ','                ','                '}';
            PointText3 = {'��״�ϰ�3��','������  ƥ��','                ','                ','                ','                ','                '}';
            PointText4 = {'��״�ϰ�4��','������  ƥ��','                ','                ','                ','                ','                '}';
            
            TotalText = [PlaneText1(:) , PlaneText2(:) , PlaneText3(:) , PlaneText4(:) , PointText1(:), PointText2(:), PointText3(:),PointText4(:)];
            
            for i = 1 : length(obj.US)
                %i
                ObstacleData = obj.US(i).ObstacleData_s;
                if isempty(ObstacleData) == 0
                    [m,~] = size(ObstacleData);
                    for Counter = 1:m
                        Data = ObstacleData(Counter,:);
                        if Data.Visible
                            Text1 = num2str(i);%���������
                            Text2 = num2str(Data.Matched);%�Ƿ�ƥ����
                            Changer = strcat({'   '},Text1,{'          '},Text2);
                            %Changer = Changer';
                            ObstacleNumber = Data.Map_Number;
                            if(obj.User_ShowObject)
                                if strcmp('Plane',Data.Type)
                                    TotalText(i+2,ObstacleNumber) = Changer;
                                else
                                    TotalText(i+2,ObstacleNumber+4) = Changer;
                                end
                            end
                        end
                    end
                end
            end
            set(obj.Obstacle_Plane_TextBox_1,'String',TotalText(:,1));
            set(obj.Obstacle_Plane_TextBox_2,'String',TotalText(:,2));
            set(obj.Obstacle_Plane_TextBox_3,'String',TotalText(:,3));
            set(obj.Obstacle_Plane_TextBox_4,'String',TotalText(:,4));
            
            set(obj.Obstacle_Point_TextBox_1,'String',TotalText(:,5));
            set(obj.Obstacle_Point_TextBox_2,'String',TotalText(:,6));
            set(obj.Obstacle_Point_TextBox_3,'String',TotalText(:,7));
            set(obj.Obstacle_Point_TextBox_4,'String',TotalText(:,8)); 
        end
        
        function Temp_ObservationShow(obj)
            %�����÷�����������ʾ��ǰ��ObstacleData_s�Ƿ���������
            for i = 1 : length(obj.US)
                %i
                ObstacleData = obj.US(i).ObstacleData_s;
                if isempty(ObstacleData) == 0
                    [m,~] = size(ObstacleData);
                    for Counter = 1:m
                        Data = ObstacleData(Counter,:);
                        if Data.Visible
                            %Data.Sensor_Number
                            %Data.Pseudo_observation
                            %Data.Diff_Pseudo_observation
                             %disp('Innovation=');
                             %Data.Innovation
%                             disp('Innovation_Variance=');
%                             Data.Innovation_Variance
                             %disp('MatchedData=');
                             %Data.MatchedData
                            %obj.US(i).noise_model.Noise
                            %obj.Pose
                            %obj.Variance
                            %obj.Variance_k_1
                        end
                    end
                end
            end
            %obj.Pose
            %obj.Pose_k_1
            %Temp = 0;
            obj.Variance
        end
        
        function ShowPredictRobot(obj)
            %��ʾ�����˵�Ԥ��λ��
        end
        
    end
     
end

