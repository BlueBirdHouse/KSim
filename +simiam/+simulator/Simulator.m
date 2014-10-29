classdef Simulator < handle
%% SIMULATOR is responsible for stepping the program through the simulation.
%
% Simulator Properties:
%   parent          - AppWindow graphics handle
%   clock           - Global timer for the simulation
%   time_step       - Time step for the simulation
%   split           - Split between calls to step()
%
% Simulator Methods:
%   step            - Executes one time step of the simulation.
%   start           - Starts the simulation.
%   stop            - Stops the simulation.

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software

    properties
        %% PROPERTIES
        
        parent          % AppWindow graphics handle
        clock           % Global timer for the simulation
        time_step       % Time step for the simulation
        
        world           % A virtual world for the simulator
        physics
        
        from_simulink
    end
    
    methods
        %% METHODS
        
        function obj = Simulator(parent, world, time_step, from_simulink)
        %% SIMULATOR Constructor
        %   obj = Simulator(parent, time_step) is the default constructor
        %   that sets the graphics handle and the time step for the
        %   simulation.
        
            obj.parent = parent;
            obj.time_step = time_step;
            if(~from_simulink)
                obj.clock = timer('Period', obj.time_step, ...
                                  'TimerFcn', @obj.step, ...
                                  'ExecutionMode', 'fixedRate');
            else
                obj.clock = [];
            end
            obj.world = world;
            obj.physics = simiam.simulator.Physics(world);
            obj.from_simulink = from_simulink;
        end
        
        function step(obj, src, event)
            %生成一步一步模拟的主函数
        %% STEP Executes one time step of the simulation.
        %   step(obj, src, event) is the timer callback which is executed
        %   once every time_step seconds.
            
            if(obj.from_simulink)
                split = obj.time_step;
                %从Simulink里面调入步长时间
            else
                split = max(obj.time_step,get(obj.clock, 'InstantPeriod'));
            end
%             split = obj.time_step;
%             fprintf('***TIMING***\nsimulator split: %0.3fs, %0.3fHz\n', split, 1/split);
            
%             tstart = tic;
            nRobots = length(obj.world.robots);
            for k = 1:nRobots
                robot_s = obj.world.robots.elementAt(k);
                
                %这里的“supervisor”其实什么也不做，在“Supervisor.m”里面
                robot_s.supervisor.execute(split);
                %就是把下一次的机器人位置设置在显示屏上即可！
                [x, y, theta] = robot_s.robot.update_state(robot_s.pose, split).unpack();
                robot_s.pose.set_pose([x, y, theta]);
            end
%             fprintf('controls: %0.3fs\n', toc(tstart));
            
%             tstart = tic;
            if obj.from_simulink
                % skip
            else
                anApp = obj.world.apps.elementAt(1);
                anApp.run(split);
            end
%             fprintf('app: %0.3fs\n', toc(tstart));
            
            %检查是否有碰撞和红外线传感器接近等问题
            bool = obj.physics.apply_physics();
            
%             tstart = tic;
            %更新显示参数
            obj.parent.ui_update(split, bool);
            drawnow;
%             fprintf('ui: %0.3fs\n', toc(tstart));
        end
        
        function start(obj)
        %% START Starts the simulation.
            if(~obj.from_simulink)
                start(obj.clock);
            end
        end
        
        function stop(obj)
        %% STOP Stops the simulation.
            if(~obj.from_simulink)
                stop(obj.clock);
            end
        end
        
        function shutdown(obj)
            if(~obj.from_simulink)
                obj.stop();
                delete(obj.clock);
            end
        end
    end
    
end