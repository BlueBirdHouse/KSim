classdef Khepera3_US_Noise < handle
    
% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software
    properties (Constant = true)
        %加性噪声和乘性噪声参数
        %加性噪声单位是米，乘性噪声没有单位
        %mean_Add = -0.0909;
        mean_Add = 0;
        standard_deviation_Add = sqrt(2.9678e-4);
        %standard_deviation_Add = 0;

        %mean_Multiplication = 0.0472;
        mean_Multiplication = 0;
        %standard_deviation_Multiplication = sqrt(2.1099e-6);
        standard_deviation_Multiplication = 0;
    end
    
    properties
        RealData %被测量的真实数据
        Sensor_Number %传感器标号
        
        %理论上噪声均值和方差
        Noise
        Mean_Noise
        variance_Noise %注意，这里是方差了
    end
    
    methods
        function obj = Khepera3_US_Noise(Sensor_Number)
            obj.Sensor_Number = Sensor_Number;
        end
        
%         function obj = GaussianNoise(mean, sigma)
%             obj.mean = mean;
%             obj.standard_deviation = sigma;
%         end
        
        function data_with_noise = apply_noise(obj,data)
            obj.RealData = data;
            [m,n] = size(obj.RealData);
            Noise_Multiplication = normrnd(obj.mean_Multiplication,obj.standard_deviation_Multiplication,[m n]);
            Noise_Add = normrnd(obj.mean_Add,obj.standard_deviation_Add,[m n]);
            
            noise = obj.RealData * Noise_Multiplication + Noise_Add;
            obj.Noise = noise;
            data_with_noise = obj.RealData + noise;
            
            %下面计算理论上噪声均值和方差
            obj.Mean_Noise = obj.RealData * obj.mean_Multiplication + obj.mean_Add;
            obj.variance_Noise = (obj.RealData * obj.standard_deviation_Multiplication)^2 + (obj.standard_deviation_Add)^2;
        end
    end
    
end

