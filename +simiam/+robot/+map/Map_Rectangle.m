classdef Map_Rectangle < handle
    %UNTITLED 此处显示有关此类的摘要
    %   这个地图是
    
     properties (Constant = true)
         %环境单位是米
         Environment_Length = 2.680;
         Environment_Width = 1.791;
         %原点是环境中心
         %Planes = [P_R P_Theta P_V]
         Obstacle_Planes = [1.791/2 0 1 1; 2.680/2 -pi/2 1 2; 1.791/2 pi 1 3; 2.680/2 pi/2 1 4];
         %Obstacle_Planes
         %Point = [P_x P_y]
         Obstacle_Point = [1.791/2 2.680/2 1; 1.791/2 -2.680/2 2; -1.791/2 -2.680/2 3;-1.791/2 2.680/2 4];
         %Obstacle_Point
         %设定可见角
         Planes_Beta = deg2rad(30);
         %Planes_Beta = deg2rad(120);
         Point_Beta = deg2rad(30);
         %Point_Beta = deg2rad(120);
     end
     
    properties
        %这个地图是供哪个机器人使用的
        Robot
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
        function obj = Map_Rectangle(robot)
            obj.Robot = robot;
        end
    end
    
end

