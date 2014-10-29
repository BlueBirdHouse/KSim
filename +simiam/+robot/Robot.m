classdef Robot < simiam.ui.Drawable

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software
    
%基本机器人定义文件，只是起一个模板作用
%类Khepera3才是一个被使用的机器人

    properties
        supervisor
        Filter
    end
    
    methods
        function obj = Robot(parent, start_pose)
            obj = obj@simiam.ui.Drawable(parent, start_pose);
        end
        
        function attach_supervisor(obj, supervisor)
            obj.supervisor = supervisor;
            supervisor.attach_robot(obj);
        end
    end
    
end

