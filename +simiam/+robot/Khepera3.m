classdef Khepera3 < simiam.robot.Robot

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software
    %真正被使用的机器人，Khepera3才是一个被使用的机器人，而不是Robot类
    %The simiam.robot.dynamics and simiam.robot.sensor subpackages define the different sensors and dynamics that a robot can use.
    
    properties
        wheel_radius
        wheel_base_length
        ticks_per_rev
        speed_factor
        
        firmware_3_0_plus = true;
        
        %定义使用的编码器和传感器
        encoders = simiam.robot.sensor.WheelEncoder.empty(1,0);
        ir_array = simiam.robot.sensor.ProximitySensor.empty(1,0);
                
        dynamics
        
        %显示这个机器人在模拟器里面的位置
        robotKhepera3_pose
    end
    
    properties (SetAccess = private)
        right_wheel_speed
        left_wheel_speed

    end
    
    methods
        function obj = Khepera3(parent, pose)
           obj = obj@simiam.robot.Robot(parent, pose);
           
           % Add surfaces: Khepera3 in top-down 2D view
           k3_top_plate =  [ -0.038   0.043    1;
                             -0.038  -0.043    1;
                              0.033  -0.043    1;
                              0.052  -0.021    1;
                              0.057       0    1;
                              0.052   0.021    1;
                              0.033   0.043    1];
                          
           k3_base =  [ -0.025   0.063    1;
                         0.030   0.063    1;
                         0.053   0.043    1;
                         0.070   0.010    1;
                         0.070  -0.010    1;
                         0.053  -0.043    1;
                         0.030  -0.063    1;
                        -0.025  -0.063    1;
                        -0.044  -0.043    1;
                        -0.052  -0.010    1;
                        -0.052   0.010    1;
                        -0.044   0.043    1];
            
            obj.add_surface(k3_base, [ 0.8 0.8 0.8 ]);
            obj.add_surface(k3_top_plate, [ 0.0 0.0 0.0 ]);
            
            %哈哈，改过来了！
            % Add sensors: wheel encoders and IR proximity sensors
            obj.wheel_radius = 0.02049915667;              % 41mm
            obj.wheel_base_length = 0.08841;        % 88.41mm
            
            %obj.firmware_3_0_plus = false;
            obj.firmware_3_0_plus = true;
            
            if (obj.firmware_3_0_plus)
                obj.ticks_per_rev = 4198;
                obj.speed_factor = 1/218.72/1000;
            else
                obj.ticks_per_rev = 2764;               
                obj.speed_factor = 1/144.01/1000;       
            end
            
            obj.encoders(1) = simiam.robot.sensor.WheelEncoder('right_wheel', obj.wheel_radius, obj.wheel_base_length, obj.ticks_per_rev);
            obj.encoders(2) = simiam.robot.sensor.WheelEncoder('left_wheel', obj.wheel_radius, obj.wheel_base_length, obj.ticks_per_rev);
            
            import simiam.robot.sensor.ProximitySensor;
            import simiam.robot.sensor.UltrasonicSensor;
            import simiam.robot.Khepera3;
            import simiam.ui.Pose2D;
            
            %这里加了专门的噪声！
            %noise_model = simiam.robot.sensor.noise.GaussianNoise(0,0.005);
            noise_model = simiam.robot.sensor.noise.GaussianNoise(0,0);
            
            %这些参数与论文上面的也不同！改过来了！
            
            ir_pose = Pose2D(-0.038, 0.049, Pose2D.deg2rad(128));
            obj.ir_array(1) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(20), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            
            ir_pose = Pose2D(0.017, 0.063, Pose2D.deg2rad(75));
            obj.ir_array(2) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(20), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            
            ir_pose = Pose2D(0.051, 0.045, Pose2D.deg2rad(42));
            obj.ir_array(3) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(20), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            
            ir_pose = Pose2D(0.067, 0.015, Pose2D.deg2rad(13));
            obj.ir_array(4) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(20), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            
            ir_pose = Pose2D(0.067, -0.015, Pose2D.deg2rad(-13));
            obj.ir_array(5) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(20), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            
            ir_pose = Pose2D(0.051, -0.045, Pose2D.deg2rad(-42));
            obj.ir_array(6) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(20), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            
            ir_pose = Pose2D(0.017, -0.063, Pose2D.deg2rad(-75));
            obj.ir_array(7) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(20), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            
            ir_pose = Pose2D(-0.038, -0.049, Pose2D.deg2rad(-128));
            obj.ir_array(8) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(20), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            
            ir_pose = Pose2D(-0.052, 0.000, Pose2D.deg2rad(180));
            obj.ir_array(9) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(20), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            
            % Add dynamics: two-wheel differential drive
            obj.dynamics = simiam.robot.dynamics.DifferentialDrive(obj.wheel_radius, obj.wheel_base_length);
            
            obj.right_wheel_speed = 0;
            obj.left_wheel_speed = 0;
            
            %在构成函数里面初始化pose
            obj.robotKhepera3_pose = pose;
            
            %添加可以被显示的超声波传感器
            for counter = 1:5
                ModelUltrasonicSensor = simiam.robot.sensor.UltrasonicSensor(1);
                ir_pose = ModelUltrasonicSensor.location_List(counter,:);
                ir_pose = Pose2D(ir_pose(1,1), ir_pose(1,2), ir_pose(1,3));
                obj.ir_array(counter+9) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.02, 0.2, Pose2D.deg2rad(30), 'simiam.robot.Khepera3.ir_distance_to_raw', noise_model);
            end
            %测试使用EKF
            obj.Filter = simiam.robot.Filters.EKF('Khepera3',[pose.x pose.y pose.theta]',zeros(3,3),obj.wheel_radius,obj.wheel_base_length);

        end
        
        function ir_distances = get_ir_distances(obj)
            ir_array_values = obj.ir_array.get_range();
            ir_distances = 0.02-log(ir_array_values/3960)/30;
        end
        
        
        function pose = update_state(obj, pose, dt)
            %根据当前位置和左右轮的速度计算下一步的位置
            sf = obj.speed_factor;
            R = obj.wheel_radius;
            
            vel_r = obj.right_wheel_speed*(sf/R);     % mm/s
            vel_l = obj.left_wheel_speed*(sf/R);      % mm/s
            
            %在这里将滤波器的系统噪声加入到其中
            SYS_Noise = obj.Filter.Q;
            pose.x = pose.x + normrnd(0,sqrt(SYS_Noise(1,1)),[1 1]);
            pose.y = pose.y + normrnd(0,sqrt(SYS_Noise(2,2)),[1 1]);
            pose.theta = pose.theta + normrnd(0,sqrt(SYS_Noise(3,3)),[1 1]);
            
            %这里预测下一步动态
            pose = obj.dynamics.apply_dynamics(pose, dt, vel_r, vel_l);
            obj.update_pose(pose);%这是Doawable属性，目的是将物理计算的结果反映到图像演示当中去
            
            for k=1:length(obj.ir_array)
                obj.ir_array(k).update_pose(pose);
            end
            
            % update wheel encoders
            sf = obj.speed_factor;
            R = obj.wheel_radius;
            
            vel_r = obj.right_wheel_speed*(sf/R); %% mm/s
            vel_l = obj.left_wheel_speed*(sf/R); %% mm/s
            
            obj.encoders(1).update_ticks(vel_r, dt);
            obj.encoders(2).update_ticks(vel_l, dt);
            
            obj.robotKhepera3_pose = pose;
            
            %下面加入滤波器运算逻辑
            %首先生成滤波器需要的控制信号：
            EKF_u = obj.Filter.Prediction_u_Maker(dt,vel_r,vel_l);%产生滤波器需要使用的驱动信息
            obj.Filter.Prediction_Pose_EKF(EKF_u);%预测位姿
            obj.Filter.Prediction_Variance_EKF();%预测方差
            obj.Filter.Obstacle_Visible(pose);%利用预测查询地图，观测那些障碍物可见
            obj.Filter.Pseudo_observation(pose);%根据预测的可见障碍物，产生伪观测信息
            obj.Filter.Guess_Data_MeanVariance();%根据伪观测信息，产生应有的噪声信息（包括模拟需要的噪声）
            obj.Filter.InnovationMaker(true);%true:产生用于模拟的新息
            obj.Filter.Match(true);%做匹配操作
            obj.Filter.Estimate();%生成估计
            
            obj.Filter.Temp_ObservationShow();%显示观测数据库信息
            obj.Filter.ShowObject();%在屏幕上打印观测到的障碍物
            obj.Filter.CleanAllObstacleData();%清楚掉ObstacleData
            
            Temp = 0;
        end
        
        function set_wheel_speeds(obj, vel_r, vel_l)
            [vel_r, vel_l] = obj.limit_speeds(vel_r, vel_l);
            
            sf = obj.speed_factor;
            R = obj.wheel_radius;
            
            obj.right_wheel_speed = floor(vel_r*(R/sf));
            obj.left_wheel_speed = floor(vel_l*(R/sf));
        end
        
        function [vel_r, vel_l] = limit_speeds(obj, vel_r, vel_l)
            % actuator hardware limits
            
            %[v,w] = obj.dynamics.diff_to_uni(vel_r, vel_l);
%             v = max(min(v,0.314),-0.3148);
%             w = max(min(w,2.276),-2.2763);
%             [vel_r, vel_l] = obj.dynamics.uni_to_diff(v,w);

            sf = obj.speed_factor;
            R = obj.wheel_radius;
            
            %我是3.0的版本，我是43000
            %max_vel = 48000*(sf/R);
            max_vel = 43000*(sf/R);
            
            vel_r = max(min(vel_r, max_vel), -max_vel);
            vel_l = max(min(vel_l, max_vel), -max_vel);
        end
    end
    
    methods (Static)
        function raw = ir_distance_to_raw(varargin)
            distance = cell2mat(varargin);
            if(distance < 0.02)
                raw = 3960;
            else
                raw = ceil(3960*exp(-30*(distance-0.02)));
            end
        end
    end
    
end

