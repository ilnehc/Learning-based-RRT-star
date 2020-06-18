ObstList = []; % Obstacle point list
for i = -15:15
    ObstList(end+1,:) = [i,20];
end
for i = -2:2
    ObstList(end+1,:) = [i, 0];
end
for i = -15:-2
    ObstList(end+1,:) = [i, 7];
end
for i = 2:15
    ObstList(end+1,:) = [i, 7];
end
for i = 0:7
    ObstList(end+1,:) = [2, i];
end
for i = 0:7
    ObstList(end+1,:) = [-2, i];
end

ObstLine = []; % Park lot line for collision check
tLine = [-15, 20 , 15, 20]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
tLine = [-15, 7, -2, 7];
ObstLine(end+1,:) = tLine;
tLine = [-2, 7, -2, 0];
ObstLine(end+1,:) = tLine;
tLine = [-2, 0, 2, 0];
ObstLine(end+1,:) = tLine;
tLine = [2, 0, 2, 7];
ObstLine(end+1,:) = tLine;
tLine = [2, 7, 15, 7];
ObstLine(end+1,:) = tLine;
tLine = [-15, 7, -15, 20];
ObstLine(end+1,:) = tLine;
tLine = [15, 7, 15, 20];
ObstLine(end+1,:) = tLine;

Vehicle.WB = 3.7;  % [m] wheel base: rear to front steer
Vehicle.W = 2.6; % [m] width of vehicle
Vehicle.LF = 4.5; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.0; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.6; % [rad] maximum steering angle 
Vehicle.MIN_CIRCLE = Vehicle.WB/tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius

% ObstList and ObstLine
Configure.ObstList = ObstList;
Configure.ObstLine = ObstLine;

% Motion resolution define
Configure.MOTION_RESOLUTION = 0.1; % [m] path interporate resolution
Configure.N_STEER = 20.0; % number of steer command
Configure.EXTEND_AREA = 0; % [m] map extend length
Configure.XY_GRID_RESOLUTION = 2.0; % [m]
Configure.YAW_GRID_RESOLUTION = deg2rad(15.0); % [rad]
% Grid bound
Configure.MINX = min(ObstList(:,1))-Configure.EXTEND_AREA;
Configure.MAXX = max(ObstList(:,1))+Configure.EXTEND_AREA;
Configure.MINY = min(ObstList(:,2))-Configure.EXTEND_AREA;
Configure.MAXY = max(ObstList(:,2))+Configure.EXTEND_AREA;
Configure.MINYAW = -pi;
Configure.MAXYAW = pi;
% Cost related define
Configure.SB_COST = 0; % switch back penalty cost
Configure.BACK_COST = 1.5; % backward penalty cost
Configure.STEER_CHANGE_COST = 1.5; % steer angle change penalty cost
Configure.STEER_COST = 1.5; % steer angle change penalty cost
Configure.H_COST = 10; % Heuristic cost

Start = [10, 9, pi];
End = [0, 6, -pi/2];
ObstMap = GridAStar(Configure.ObstList,End,Configure.XY_GRID_RESOLUTION);
Configure.ObstMap = ObstMap;
% cla % 
% 
% ------------------------------------------
% uncomment this to generate dataset
% ------------------------------------------
% idx = 0;
% for startx = 0:12
%     for starty = 10:16
%         Start = [startx, starty, 0];
%         [x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure);
%         if isempty(x)
%             disp("Failed to find path!")
%         else
%             idx
%             various = {'x','y','theta','direction','delta'};
%             various2 = {'start','end'};
%             name1 = ['./output/route' mat2str(idx) '.csv'];
%             name2 = ['./output/startend' mat2str(idx) '.csv'];
%             result_table = table(x', y', th', D', delta', 'VariableNames',various);
%             result_table2 = table(Start', End', 'VariableNames',various2);
%             writetable(result_table, name1)
%             writetable(result_table2, name2)
%             idx = idx + 1;
%         end
% 
%         Start = [-startx, starty, pi];
%         [x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure);
%         if isempty(x)
%             disp("Failed to find path!")
%         else
%             idx
%             various = {'x','y','theta','direction','delta'};
%             various2 = {'start','end'};
%             name1 = ['./output/route' mat2str(idx) '.csv'];
%             name2 = ['./output/startend' mat2str(idx) '.csv'];
%             result_table = table(x', y', th', D', delta', 'VariableNames',various);
%             result_table2 = table(Start', End', 'VariableNames',various2);
%             writetable(result_table, name1)
%             writetable(result_table2, name2)
%             idx = idx + 1;
%         end
%     end
% end
% ------------------------------------------
    
% Single run testing
[x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure);
GridAStar(ObstList,End,2);
if isempty(x)
    disp("Failed to find path!")
else
    VehicleAnimation(x,y,th,Configure,Vehicle)
end
% various = {'x','y','theta','direction','delta'};
% various2 = {'start','end'};
% result_table = table(x', y', th', D', delta', 'VariableNames',various);
% result_table2 = table(Start', End', 'VariableNames',various2);
% writetable(result_table, 'route.csv')
% writetable(result_table2, 'startend.csv')



