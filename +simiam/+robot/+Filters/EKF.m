classdef EKF < handle
    %UNTITLED 此处显示有关此类的摘要
    %   注意，滤波器只能够被初始化一次！
    
     properties (Constant = true)
        %系统方差阵
        Q = diag([(5e-3)^2 (5e-3)^2 (deg2rad(0.4))^2]); %5cm 和 4度
        
        %初始化地图
        Map = simiam.robot.map.Map_Rectangle('Khepera3');
        
     end
    
    properties
        type %那种型号的机器人在使用滤波器
        
        Pose %机器人的位姿估计
        Diff_Pose %系统方程的导数
        Pose_k_1 %机器人的位姿预测
        
        Variance %方差和预测方差
        Variance_k_1
        
        IF_Observation_Success %表示观测及新息的产生是否成功
        IF_Filter_Success %这一次的滤波结果是否成功
        MatchedDataNumber = 0; %表示匹配上的有效数据个数
        
        %机器人结构参数
        r_R
        r_L
        b
        
        %控制信号U
        u 

        %初始化传感器
        US = simiam.robot.sensor.UltrasonicSensor.empty(1,0);
        
        %下面是显示灯塔的4*2个图片框
        Obstacle_Plane_TextBox_1
        Obstacle_Plane_TextBox_2
        Obstacle_Plane_TextBox_3
        Obstacle_Plane_TextBox_4
        
        Obstacle_Point_TextBox_1
        Obstacle_Point_TextBox_2
        Obstacle_Point_TextBox_3
        Obstacle_Point_TextBox_4
        
        
        %显示是否需要显示观测过程
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
            %用来修正过大的角度，输入域是：-pi-pi
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
                %如果是K3机器人，则启动K3相关的初始化工作
                %注意，这个函数在循环当中只能够被调用一次
                obj.IF_Filter_Success = false;
                
                %初始化位姿和位姿的预测
                obj.Pose = pose;
                obj.Pose_k_1 = pose;
                
                %初始化位姿方差和位姿方差的预测
                obj.Variance = Variance;
                obj.Variance_k_1 = Variance;
                
                %初始化结构参数
                obj.r_R = wheel_radius;
                obj.r_L = wheel_radius;
                obj.b = wheel_base_length;
                
                %初始化传感器
                for i = 1:5
                    obj.US(i) = simiam.robot.sensor.UltrasonicSensor(i);
                end
                  
                %在监视窗口上初始化几个文本框用来显示传感器状态
                obj.Obstacle_Plane_TextBox_1 = annotation('textbox',...
                    [0.8 0.6 0.05 0.05],...
                    'String',{'平面障碍1：','传感器  匹配','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Plane_TextBox_2 = annotation('textbox',...
                    [0.45 0.23 0.05 0.05],...
                    'String',{'平面障碍2：','传感器  匹配','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Plane_TextBox_3 = annotation('textbox',...
                    [0.05 0.6 0.05 0.05],...
                    'String',{'平面障碍3：','传感器  匹配','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Plane_TextBox_4 = annotation('textbox',...
                    [0.45 0.89 0.05 0.05],...
                    'String',{'平面障碍4：','传感器  匹配','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Point_TextBox_1 = annotation('textbox',...
                    [0.8 0.89 0.05 0.05],...
                    'String',{'点状障碍1：','传感器  匹配','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Point_TextBox_2 = annotation('textbox',...
                    [0.8 0.23 0.05 0.05],...
                    'String',{'点状障碍2：','传感器  匹配','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Point_TextBox_3 = annotation('textbox',...
                    [0.05 0.23 0.05 0.05],...
                    'String',{'点状障碍3：','传感器  匹配','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                obj.Obstacle_Point_TextBox_4 = annotation('textbox',...
                    [0.05 0.89 0.05 0.05],...
                    'String',{'点状障碍4：','传感器  匹配','   1          1','   2          1','   3          1','   4          1','   5          1'},...
                    'FitBoxToText','on','Color',[0.600000023841858 0.200000002980232 0],'FontWeight','bold','LineStyle','none');
                
                Temp = 0;
            end
        end
        
        function Prediction_Pose_EKF(obj,u)
            %执行滤波器预测
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
            %生成滤波算法需要的特殊的u
            R = (obj.r_R+obj.r_L)/2;
            L = obj.b;
            
            v = R/2*(vel_r+vel_l);
            w = R/L*(vel_r-vel_l);
            
            T_k = v * dt;
            Det_Theta_k = w * dt;
            
            u = [T_k ; Det_Theta_k];
        end
                
        function Prediction_Variance_EKF(obj)
            %改方法必须在调用Prediction_Pose_EKF以后调用
            Diff = obj.Diff_Pose;
            P = obj.Variance;
            
            obj.Variance_k_1 = Diff * P * (Diff)' + obj.Q;
        end
        
        %下面的函数处理观测部分
        function Obstacle_Visible(obj,Realpose)
            %检测传感器可见的障碍物信息，并填充ObstacleData
            %这里面的角度都要使用2pi制度！
%             x_k_1 = obj.Pose_k_1(1,1);
%             y_k_1 = obj.Pose_k_1(2,1);
%             theta_k_1 = obj.Pose_k_1(3,1);
            
            x_k_1 = Realpose.x;
            y_k_1 = Realpose.y;
            theta_k_1 = Realpose.theta;
            theta_k_1 = obj.FixAngle(theta_k_1);
            
            for i = 1:length(obj.US)
                %逐个检查平面目标的可见性
                Obstacle_Planes = obj.Map.Obstacle_Planes;
                [m,~] = size(Obstacle_Planes);
                for j = 1:m
                    %取出地图障碍物数据
                    Phi_st = Obstacle_Planes(j,2);
                    Beta_t = obj.Map.Planes_Beta;
                    
                    %当前传感器在世界坐标系下的转角
                    a_st = theta_k_1 + obj.US(i).location(3,1);
                    
                    %修正角度为2PI制度
                    Phi_st = obj.Pi_Pi22PI(Phi_st);
                    Beta_t = obj.Pi_Pi22PI(Beta_t);
                    a_st = obj.Pi_Pi22PI(a_st);
                    
                    %这里的角度判断会在Angile1，Angile2经过0度的时候发生错误
                    Angile1 = Phi_st - Beta_t/2;
                    Angile2 = Phi_st + Beta_t/2;
                    %Angile2 = obj.Pi_Pi22PI(Phi_st + Beta_t/2);
                    
                    %判断是否是同号
                    SameSign = Angile1 * Angile2;
                    
                    Angile1 = obj.Pi_Pi22PI(Angile1);
                    Angile2 = obj.Pi_Pi22PI(Angile2);
                    
                    if(SameSign >= 0)
                        Condition = ((Angile1 <= a_st) && (Angile2 >= a_st));
                    else
                        Condition = ((Angile1 <= a_st) || (Angile2 >= a_st));
                    end
                    
                    if(Condition)
                        %说明这个障碍物可见，拷贝有关改障碍物的信息到ObstacleData
                        Temp = simiam.robot.sensor.data.ObstacleData(i);
                        Temp.Type = 'Plane';
                        Temp.Visible = true;
                        Temp.Map_Number = j;
                        
                        obj.US(i).ObstacleData_s = [obj.US(i).ObstacleData_s ; Temp];
                    end
                end
            end    
            
              %逐个检查点目标的可见性
             for i = 1:length(obj.US)
                 Obstacle_Points = obj.Map.Obstacle_Point;
                 [m,~] = size(Obstacle_Points);
                 for j = 1:m
                     p_x = Obstacle_Points(j,1);
                     p_y = Obstacle_Points(j,2);
                     
                     %调出传感器的机器人上坐标
                     x_s_ = obj.US(i).location(1,1);
                     y_s_ = obj.US(i).location(2,1);
                     Temp = [x_s_ y_s_ 1]';
                     Temp = obj.get_transformation_matrix(x_k_1,y_k_1,theta_k_1) * Temp;
                     
                     x_s = Temp(1,1);
                     y_s = Temp(2,1);
                     
                     Phi_st = atan2(p_y - y_s, p_x - x_s);
                     
                     Beta_t = obj.Map.Point_Beta;
                     a_st = theta_k_1 + obj.US(i).location(3,1);
                     
                     %修正角度为2PI制度
                     Phi_st = obj.Pi_Pi22PI(Phi_st);
                     Beta_t = obj.Pi_Pi22PI(Beta_t);
                     a_st = obj.Pi_Pi22PI(a_st);
                     
                     %这里的角度判断会在Angile1，Angile2经过0度的时候发生错误
                     Angile1 = Phi_st - Beta_t/2;
                     Angile2 = Phi_st + Beta_t/2;
                     %Angile2 = obj.Pi_Pi22PI(Phi_st + Beta_t/2);
                    
                     %判断是否是同号
                     SameSign = Angile1 * Angile2;
                    
                     Angile1 = obj.Pi_Pi22PI(Angile1);
                     Angile2 = obj.Pi_Pi22PI(Angile2);
                    
                     if(SameSign >= 0)
                         Condition = ((Angile1 <= a_st) && (Angile2 >= a_st));
                     else
                         Condition = ((Angile1 <= a_st) || (Angile2 >= a_st));
                     end
                     
                     if(Condition)
                         %说明这个障碍物可见，拷贝有关改障碍物的信息到ObstacleData
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
            %逐个检查ObstacleData_s，并填充伪观测信息
            for i = 1:length(obj.US)
                %首先取出需要处理的ObstacleData_s数组
                ObstacleData = obj.US(i).ObstacleData_s;
                if isempty(ObstacleData) == 0
                    [m,~] = size(ObstacleData);%测量维数
                    for Counter = 1:m
                        Data = ObstacleData(Counter,:);
                        if Data.Visible
                            if strcmp('Plane',Data.Type) %根据平面目标和点目标做分别处理
                                %提取相应的地图信息
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
                                %提取相应的地图信息
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
                        %写回数据
                        ObstacleData(Counter,:) = Data;
                    end
                end
                %写回数据
                obj.US(i).ObstacleData_s = ObstacleData;
                
            end
        end
        
        function Guess_Data_MeanVariance(obj)
            %逐个检查ObstacleData_s，并猜测这个噪声的均值和方差
            for i = 1:length(obj.US)
                %首先取出需要处理的ObstacleData_s数组
                ObstacleData = obj.US(i).ObstacleData_s;
                if isempty(ObstacleData) == 0
                    [m,~] = size(ObstacleData);%测量维数
                    for Counter = 1:m
                        Data = ObstacleData(Counter,:);
                        if Data.Visible
                            %首先取出伪观测信息
                            Pseudo_observ = Data.Pseudo_observation;
                            Data.Pseudo_observationWithNoise = obj.US(i).noise_model.apply_noise(Pseudo_observ);
                            %填写噪声信息
                            Data.MatchedData_Mean = obj.US(i).noise_model.Mean_Noise;
                            Data.MatchedData_Variance = obj.US(i).noise_model.variance_Noise;
                            
                        end
                        %写回数据
                        ObstacleData(Counter,:) = Data;
                    end
                end
                %写回数据
                obj.US(i).ObstacleData_s = ObstacleData;
            end
        end
        
        function InnovationMaker(obj,IsSimulation)
            if(IsSimulation)
                %如果没有任何一个有效观测的话，那么这一次滤波就是失败的
                obj.IF_Observation_Success = false;
                for i = 1:length(obj.US)
                    %首先取出需要处理的ObstacleData_s数组
                    ObstacleData = obj.US(i).ObstacleData_s;
                    if isempty(ObstacleData) == 0
                        [m,~] = size(ObstacleData);%测量维数
                        for Counter = 1:m
                            Data = ObstacleData(Counter,:);
                            if Data.Visible
                                %只要有一个有效观测，那么就说明这一次滤波是成功的
                                obj.IF_Observation_Success = true;
                                %首先取出伪观测信息
                                observation_ = Data.Pseudo_observation;
                                %由于在模拟状态下，一次测量声波的发出只能收到一个回波，那么这里观测的维数必定是相同的
                                Realobservation = Data.Pseudo_observationWithNoise;
                                %产生新息
                                Data.Innovation = Realobservation - observation_;
                            end
                            %写回数据
                            ObstacleData(Counter,:) = Data;
                        end
                    end
                    %写回数据
                    obj.US(i).ObstacleData_s = ObstacleData;
                end
            end
        end
        
        function Match(obj,IsSimulation)
            %做匹配操作
            if(IsSimulation)
                %如果没有任何一个成功匹配的话，那么这一次滤波就是失败的
                obj.IF_Filter_Success = false;
                for i = 1:length(obj.US)
                    %首先取出需要处理的ObstacleData_s数组
                    ObstacleData = obj.US(i).ObstacleData_s;
                    if isempty(ObstacleData) == 0
                        [m,~] = size(ObstacleData);%测量维数
                        for Counter = 1:m
                            Data = ObstacleData(Counter,:);
                            if Data.Visible
                                %在这里做匹配，现在是模拟，假设都匹配上了
                                Data.Matched = true;
                                %记录有多少数据匹配上了
                                obj.MatchedDataNumber = obj.MatchedDataNumber + 1;
                                obj.IF_Filter_Success = true;
                            end
                            %写回数据
                            ObstacleData(Counter,:) = Data;
                        end
                    end
                    %写回数据
                    obj.US(i).ObstacleData_s = ObstacleData;
                end
            end
        end
        
        function Estimate(obj)
            %如果没有任何一个成功匹配的话，那么这一次滤波就是失败的
            if(obj.IF_Filter_Success)
                %首先组合数据
                det_h = zeros(obj.MatchedDataNumber,3);%观测方程的导数集合
                
                V = zeros(obj.MatchedDataNumber,1);%新息的集合
                R = zeros(obj.MatchedDataNumber,obj.MatchedDataNumber); %观测噪声方差的集合，对角
                
                Printer = 1;
                
                for i = 1:length(obj.US)
                %首先取出需要处理的ObstacleData_s数组
                    ObstacleData = obj.US(i).ObstacleData_s;
                    if isempty(ObstacleData) == 0
                        [m,~] = size(ObstacleData);%测量维数
                        for Counter = 1:m
                            Data = ObstacleData(Counter,:);
                            if Data.Matched
                                %填充滤波器需要的数据
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
                %处理滤波没有成功的情况（做一次空预测）
                obj.Pose = obj.Pose_k_1;
                obj.Variance = obj.Variance_k_1;
            end
            Temp = 0;
        end
        
        function Rad_Fixed = Pi_Pi22PI(obj,Rad)
            %将-pi-pi域转化为2PI域
            Rad = obj.FixAngle(Rad);
            if(Rad < 0)
                Rad = 2*pi - abs(Rad);
            end
            Rad_Fixed = Rad;
        end

        function CleanAllObstacleData(obj)
            %一次性清除所有传感器数据
            for i = 1 : length(obj.US)
                obj.US(i).ObstacleData_Cleaner();
            end
            obj.MatchedDataNumber = 0;
        end
        
        function ShowObject(obj)
            %用来在地图上显示哪些障碍物被哪些传感器检测到了！
            PlaneText1 = {'平面障碍1：','传感器  匹配','                ','                ','                ','                ','                '}';
            PlaneText2 = {'平面障碍2：','传感器  匹配','                ','                ','                ','                ','                '}';
            PlaneText3 = {'平面障碍3：','传感器  匹配','                ','                ','                ','                ','                '}';
            PlaneText4 = {'平面障碍4：','传感器  匹配','                ','                ','                ','                ','                '}';
            
            PointText1 = {'点状障碍1：','传感器  匹配','                ','                ','                ','                ','                '}';
            PointText2 = {'点状障碍2：','传感器  匹配','                ','                ','                ','                ','                '}';
            PointText3 = {'点状障碍3：','传感器  匹配','                ','                ','                ','                ','                '}';
            PointText4 = {'点状障碍4：','传感器  匹配','                ','                ','                ','                ','                '}';
            
            TotalText = [PlaneText1(:) , PlaneText2(:) , PlaneText3(:) , PlaneText4(:) , PointText1(:), PointText2(:), PointText3(:),PointText4(:)];
            
            for i = 1 : length(obj.US)
                %i
                ObstacleData = obj.US(i).ObstacleData_s;
                if isempty(ObstacleData) == 0
                    [m,~] = size(ObstacleData);
                    for Counter = 1:m
                        Data = ObstacleData(Counter,:);
                        if Data.Visible
                            Text1 = num2str(i);%传感器标记
                            Text2 = num2str(Data.Matched);%是否匹配标计
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
            %调试用方法，用来显示当前的ObstacleData_s是否正常工作
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
            %显示机器人的预测位置
        end
        
    end
     
end

