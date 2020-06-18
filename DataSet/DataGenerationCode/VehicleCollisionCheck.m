function isCollision = VehicleCollisionCheck(pVec,ObstLine,Vehicle)
    W = Vehicle.W;
    LF = Vehicle.LF;
    LB = Vehicle.LB;
    Cornerfl = [LF, W/2];
    Cornerfr = [LF, -W/2];
    Cornerrl = [-LB, W/2];
    Cornerrr = [-LB, -W/2];
    Pos = pVec(1:2);
    theta = pVec(3);
    dcm = angle2dcm(-theta, 0, 0); 
    
    tvec = dcm*[Cornerfl';0]; 
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;   
    % Car 
    Rect = [];                            %  _ _ _ _ _
    Rect(end+1,:) = [Cornerfl, Cornerfr]; % |    ^    |
    Rect(end+1,:) = [Cornerfr, Cornerrr]; % |    ^    |
    Rect(end+1,:) = [Cornerrr, Cornerrl]; % |    ^    |
    Rect(end+1,:) = [Cornerrl, Cornerfl]; % | _ _^ _ _|
    isCollision = false;
%     ------------------------------------------
%     Scenario 1 reverse parking
%     ------------------------------------------
    obs_self_define=[-15, 20; 15, 20; 15, 7; 2, 7; 2, 0; -2, 0; -2, 7; -15, 7; -15, 20];
    [xi,yi] = polyxpoly([Rect(:,1);Rect(1,1)],[Rect(:,2);Rect(1,2)],obs_self_define(:,1),obs_self_define(:,2));
    if isempty(xi)==0
%     ------------------------------------------
%     Scenario 2 parallel parking
%     ------------------------------------------
%     obs_self_define=[-15, 13; 15, 13; 15, 3; 5, 3; 5, 0; -5, 0; -5, 3; -15, 3; -15, 13];
%     [xi,yi] = polyxpoly([Rect(:,1);Rect(1,1)],[Rect(:,2);Rect(1,2)],obs_self_define(:,1),obs_self_define(:,2));
%     if isempty(xi)==0
%     ------------------------------------------
%     Scenario 3 complex map 1 reverse parking
%     ------------------------------------------
%     obs_self_define=[-30, 35; 30, 35; 30, 7; 2, 7; 2, 0; -2, 0; -2, 7; -30, 7; -30, 35];
%     obs_self_define2 = [-12, 18; -12, 28; -10, 28; -10, 18; -12, 18];
%     obs_self_define3 = [10, 20; 10, 23; 20, 23; 20, 20; 10, 20];
%     if isempty(xi)==0 || isempty(xi2)==0 || isempty(xi3)==0
%     ------------------------------------------
%     Scenario 4 complex map 2 reverse parking
%     ------------------------------------------
%     obs_self_define=[-30, 35; 30, 35; 30, 7; 2, 7; 2, 0; -2, 0; -2, 7; -30, 7; -30, 35];
%     obs_self_define2 = [-20, 18; -20, 20; -10, 20; -10, 18; -20, 18];
%     obs_self_define3 = [15, 12; 15, 15; 25, 15; 25, 12; 15, 12];
%     obs_self_define4 = [5, 23; 5, 28; 10, 28; 10, 23; 5, 23];
%     if isempty(xi)==0 || isempty(xi2)==0 || isempty(xi3)==0 || isempty(xi4)==0
%     ------------------------------------------
        isCollision = true;
    end
end

function isCollision = RectLineCollisionCheck(Rect, Line)
    isCollision = SATCheckObj2Line(Rect, Line, Line);
    if isCollision == false
        return
    else
        isCollision = SATCheckObj2Line(Rect, Line, Rect(1,:));
        if isCollision == false
            return
        else
            isCollision = SATCheckObj2Line(Rect, Line, Rect(2,:));
        end
    end
end

function isCollision = SATCheckObj2Line(Object, workLine, refLine)
    theta = atan2(refLine(4)-refLine(2),refLine(3)-refLine(1));
    dcm = angle2dcm(theta, 0, 0);
    pStart = dcm*[workLine(1:2)'; 0];
    pEnd = dcm*[workLine(3:4)'; 0];
    LineMinx = min(pStart(1), pEnd(1));
    LineMaxx = max(pStart(1), pEnd(1));
    LineMiny = min(pStart(2), pEnd(2));
    LineMaxy = max(pStart(2), pEnd(2));
    % To find the max and min corrdination of object
    dim = size(Object);
    objLineMinx = inf;
    objLineMaxx = 0;
    objLineMiny = inf;
    objLineMaxy = 0;
    for i = 1:dim(1)
        objpStart = dcm*[Object(i,1:2) 0]';
        objpEnd = dcm*[Object(i,3:4) 0]';
        objLineMinx = min([objLineMinx, objpStart(1), objpEnd(1)]);
        objLineMaxx = max([objLineMaxx, objpStart(1), objpEnd(1)]);
        objLineMiny = min([objLineMiny, objpStart(2), objpEnd(2)]);
        objLineMaxy = max([objLineMaxy, objpStart(2), objpEnd(2)]);
    end

    isCollision = true;
    if LineMinx > objLineMaxx || LineMaxx < objLineMinx || ...
            LineMiny > objLineMaxy || LineMaxy < objLineMiny
        isCollision = false;
        return
    end
end
