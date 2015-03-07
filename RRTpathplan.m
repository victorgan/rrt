function prm = path_planner

    % Import Obstacle Locations
    filename ='Simple2Obst.txt';
    obst = importObstacles(filename);
    drawCourse(obst);

    % Set Start and Goal locations
    p_start = [0;11];
    p_goal = [0;-1];

    robot.ballradius = 0.5;
    robot.p = p_start;

    % Parameters
    param.res = 0.5;
    param.thresh = 5;
    param.maxiters = 1000;
    param.smoothiters = 150;

    % Plan the path
    pathPoints = PlanPathRRT(robot,obst,param,p_start,p_goal);

    % Smooth the path
    if (~isempty(pathPoints))
        pSmooth = SmoothPath(robot,obst,param,pathPoints)
    end

    % DRAW

    % Starting/ending positions
    plotCircle(robot.p(1,1),robot.p(2,1),robot.ballradius,'g');
    plotCircle(p_goal(1,1),p_goal(2,1),robot.ballradius,'r');

    % Plot the unsmoothed path
    for i=2:length(pathPoints)
        plot([pathPoints(1,i);pathPoints(1,i-1)],[pathPoints(2,i);pathPoints(2,i-1)],'r','LineWidth',3);
    end

    % Plot the smoothed path
    for i=2:length(pSmooth)
        plot([pSmooth(1,i);pSmooth(1,i-1)],[pSmooth(2,i);pSmooth(2,i-1)],'g','LineWidth',3);
    end
end % function


function h = plotCircle(x,y,r,color)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit,color,'LineWidth',3);
    %hold off
end % function

function rrt = AddNode(rrt,p,iPrev)
    node.p = p;
    node.iPrev = iPrev;
    rrt{end+1} = node;
end % function

function rrt = AddNodeToEnd(rrt,p)
    rrt = AddNode(rrt,p,length(rrt));
end % function

% pathPoints    2xp double  array of points in path
% robot         struct indicating ballradius and current point
% param         parameters
% p_start       2x1 double  initial configuration
% p_end         2x1 double  end point
function [pathPoints, rrt] = PlanPathRRT(robot,obst,param,p_start,p_goal)
    assert( all(size(p_goal) == size(p_start)), 'Dimension mismatch');

    global iterations
    pathPoints = [];

    rrt = {};
    rrt = AddNode(rrt,p_start,0);
    for iter = 1:param.maxiters

        % Generate random configuration
        p = rand(2,1); % random p
        p(1,1) = p(1,1)*10-5;
        p(2,1) = p(2,1)*15-4;
        qRand = p;
        robot.p = p;

        isObstacleCollision = collideWithObstacle(robot,obst);
        if isObstacleCollision % skip to next iteration
            continue
        end

        % Find the qNear, the closet vertex in rrt 
        [qNear, qNearIdx] = nearestVertex(qRand, rrt);

        invalidEdge = collideWithEdge(robot,obst,p,qNear,param.res); %check for valid edge
        if invalidEdge % skip to next iteration if not valid edge
            continue;
        end
        rrt = AddNode(rrt,p,qNearIdx); % add p to T with parent qNear

        % TODO separate RRT
        plot([p(1,1);rrt{qNearIdx}.p(1,1)],[p(2,1);rrt{qNearIdx}.p(2,1)],'m','LineWidth',3);

        distToGoal    = norm(p - p_goal);
        isCloseToGoal = (distToGoal < param.thresh);
        if isCloseToGoal
            invalidEdge = collideWithEdge(robot, obst, p, p_goal, param.res); %check for valid edge
            if invalidEdge % skip to next iteration if not valid edge
                continue;
            end

            rrt = AddNodeToEnd(rrt,p_goal);
            lastNodeIdx = length(rrt);
            pathPoints = pathFromRrt(rrt, lastNodeIdx);
            return
        end % if

    end
end % function

function pathPoints = pathFromRrt(rrt, nodeNum)
    i = nodeNum;
    pathPoints(:,1) = rrt{i}.p;
    while 1
        i = rrt{i}.iPrev;
        isStartNode = i == 0;
        if isStartNode
            break;
        end
        pathPoints = [rrt{i}.p pathPoints];
    end

end % function

function [qNear, qNearIdx] = nearestVertex(point, rrt)
    % Find the qNear, the closet vertex in rrt 
    for i=1:length(rrt)
        dist = norm(rrt{i}.p - point);
        if (i==1) || (dist < mindist)
            mindist = dist;
            qNearIdx = i;
            qNear = rrt{i}.p;
        end
    end % for
end % function

% Determine if the robot collides with any obstacles
function isObstacleCollision = collideWithObstacle(robot,obst)
    isObstacleCollision = false;
    numObstacles = length(obst.ball);
    for j=1:numObstacles % check for robot-obstacle collision
        dist = norm(robot.p-obst.ball{j}.p);
        if dist < (robot.ballradius + obst.ball{j}.r)
            isObstacleCollision = true;
            return;
        end 
    end
end % function

% Draw a line between p1 and p2. If any point on the line collides with an
% obstacle, return false.
function isObstacleCollision = collideWithEdge(robot,obst,p1,p2,res)
% TODO less parameters

    isObstacleCollision = 0;
    d = norm(p1 - p2);
    m = ceil(d/res);
    t = linspace(0,1,m);

    for i=2:(m-1)
        p = (1-t(i))*p1 + t(i)*p2; %calculate configuration
        robot.p = p;
        isObstacleCollision = collideWithObstacle(robot,obst); 
        if isObstacleCollision 
            return;
        end
    end
end % function

function P = SmoothPath(robot,obst,param,P) %was SmoothPath(robot,obst,param,m,Q)
    %
    % INPUTS
    %
    %   robot - says where the robot is
    %
    %           NOTE: to make "robot" describe the robot at configuration
    %           "something" (a column vector of length robot.n), you must call:
    %
    %             robot.q = something
    %             robot = ForwardKinematics(robot);
    %
    %   obst - says where the obstacles are
    %
    %   param - some parameters:
    %
    %    param.res - resolution with which to sample straight-line paths
    %    param.maxiters - maximum number of RRT iterations
    %    param.thresh - distance below which a configuration is considered
    %                   "close" to qgoal (called "d" in the homework)
    %    param.smoothiters - maximum number of smoothing iterations
    %
    %   qstart, qgoal - start and goal configurations
    %
    % OUTPUTS
    %
    %   Q - a path, same format as before:
    %       
    %           Q = [q1 q2 q3 ... qM]
    %
    %               where q1=qstart and qM=qgoal; in other words, the sequence
    %               of straight-line paths from q1 to q2, q2 to q3, etc., takes
    %               the robot from start to goal without collision
    %
    % YOUR CODE HERE...
    %
    P = P;
    [n,m] = size(P);
    clearvars n;
    l = zeros(m,1);
    for k=2:m
        l(k)=norm(P(:,k)-P(:,k-1)) + l(k-1); % find all of the straight-line distances
    end
    l_init = l(m);
    for iter = 1:param.smoothiters
        s1 = rand(1,1)*l(m); 
        s2 = rand(1,1)*l(m); 
        if s2 < s1
            temps = s1;
            s1 = s2;
            s2 = temps;
        end
        for k=2:m
            if s1 < l(k)
                i = k - 1;
                break;
            end
        end
        for k=(i+1):m
            if s2 < l(k)
                j = k - 1;
                break;
            end
        end
        if (j <= i)
            continue;
        end
        t1 = (s1 - l(i))/(l(i+1)-l(i));
        gamma1 = (1 - t1)*P(:,i) + t1*P(:,i+1);
        t2 = (s2 - l(j))/(l(j+1)-l(j));
        gamma2 = (1 - t2)*P(:,j) + t2*P(:,j+1);
        col = collideWithEdge(robot,obst,gamma1,gamma2,param.res); %check for valid edge
        if col == 1
            continue;
        end
        newP = [P(:,1:i) gamma1 gamma2 P(:,j+1:m)];
        clearvars P;
        P = newP;
        [n,m] = size(P);
        clearvars n;
        l = zeros(m,1);
        for k=2:m
            l(k)=norm(P(:,k)-P(:,k-1)) + l(k-1);
        end
    end % for iter

end % function

% Import
function obst = importObstacles(filename)
    % Import Obstacle Locations
    filename ='Simple2Obst.txt';
    delimiterIn = ' ';
    headerlinesIn = 0;
    rawdata = importdata(filename,delimiterIn,headerlinesIn);
    object_coord = rawdata(1:500,2);

    % Tile the obstacles with balls
    obst.ball = {};
    for i=1:2:500
        if (object_coord(i,1)<999)

            onew.p = [object_coord(i,1);object_coord(i+1,1)];
            onew.r = 1/3;
            onew.handle = [];
            obst.ball{end+1} = onew;
        end
    end
end

% Drawing
function drawCourse(obst)

    % Draw Course
    map = figure;
    course_outx=[-2,-4,-6,-6, 6,6,4,2];
    course_outy=[ 0, 0, 0,12,12,0,0,0];
    hold on;
    plot(course_outx,course_outy,'k','LineWidth',5);
    axis([-7,7,-6,13]);
    axis equal
    set(gca,'XTick',-13:1:13)
    set(gca,'YTick',-6:1:13)
    grid ON

    % Draw obstacles
    numObstacles = length(obst.ball);
    for i = 1:numObstacles
            obstacle = obst.ball{i};
            plotCircle(obstacle.p(1,1),obstacle.p(2,1),obstacle.r,'b');
    end % for i

end % function
