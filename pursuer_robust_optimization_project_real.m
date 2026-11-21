%% Coordinated Capture via Strategic Formation Command Sequence
% This script is adapted from the pursuit-evasion game developed by students in the CCRL lab 
% it emulates a capture scenario where a team of agents are
% commanded to execute a sequence of distributed formations to influence an
% evader's motion and set it up for capture.
% THIS VERSION OF CODE SHOULD BE FUNCTIONAL EVEN IF THERE ARE NOT ENOUGH
% ROBOT PLATFORMS  --- single evader only for now
% FILE CREATED: 2 SEP 2024
% LAST EDIT DATE: 25 OCT 2024 
% This version is functional with two robots (one evader one pursuer).
% TODO: Move goal/make changes to the task. Run more trials.
rng(21)


%% Load file to store position data
data = load('simulation_data.mat')

%% Initialize robots
% number of robots and dimensions
robotRadius = 0.07; % [m]
robotDiameter = 2*robotRadius;
finalStatusFlag = 1; % 0 = evader reached goal, 1 = robots left domain, 2 = evader captured
numEvaders = 1;
numPursuers = 1;
%numAgents = numEvaders - numPursuers;

% Domain definition
ft2m = 1/3.28084;
domainBoundaries = [-17/2,17/2,-12/2,12/2]*ft2m;
boundaryTol = robotRadius;

barrierSafetyDist = 2.0*robotRadius;
controlScaleFactor = 1;


% Robot speeds
pursuerMaxSpeed = 0.3; % [m/s]
evaderMaxSpeed  = 0.3; % [m/s]

angularVelLimit = pi;

% Evader strategy
evaderControl = 'stochastic'; % 'stochastic', 'Go2Goal'

% Pursuer Strategy
useParallelNav = 1;
headVec = @(th) [cos(th);sin(th)];

%% Supervisor
teamSize = 1;
numberTeams = ceil(numPursuers/teamSize);
groupSizes = teamSize*ones(numberTeams,1);
groupSizes(ceil(numberTeams/2)) = numPursuers - (numberTeams-1)*teamSize;
% DeploymentTimes
deployTimes = 0;%[0,10,0];

% Init Positions
initPursuerPose = [(domainBoundaries(1)+boundaryTol+2*robotRadius)*ones(1,numPursuers);
    linspace(domainBoundaries(3)+2*boundaryTol+3*robotRadius,domainBoundaries(4)-boundaryTol-3*robotRadius,numPursuers);
    zeros(1,numPursuers)];
initEvaderPose = [(domainBoundaries(2)-boundaryTol-2*robotRadius)*ones(1,numEvaders);
    %linspace(domainBoundaries(3)+robotRadius,domainBoundaries(4)-robotRadius,numEvaders);
    domainBoundaries(3)+boundaryTol+2*robotRadius%+rand(1)*(diff([domainBoundaries(3)+boundaryTol,domainBoundaries(4)-boundaryTol])-4*robotRadius);
    pi*ones(1,numEvaders)];
goalLocation = mean(initPursuerPose(1:2,:),2)+0*robotRadius*[1;0];

pursuerPose = initPursuerPose;
evaderPose = initEvaderPose;

%% Strategy Formation Definition using Persistent Networks
% Define the formation specification matrices.
% Line formation
%For team of pursuers:
% sepBetweenRobots = 4*robotRadius;
% lineSep = 2*robotRadius + sepBetweenRobots;
% Dline = lineSep*toeplitz(0:teamSize-1);
% formationNames = {'/','\','<','>','|'};

%Pursuer strategy
teamTargets = 2;
% Target positions
pursuerTargetPositions =  [(domainBoundaries(2)-2*robotRadius)*ones(1,numberTeams);
    %linspace(domainBoundaries(3)+robotRadius,domainBoundaries(4)-robotRadius,numEvaders);
    linspace(domainBoundaries(3)+2*robotRadius+lineSep,domainBoundaries(4)-2*robotRadius-lineSep,numberTeams)];
pursuerTargetPositions = [pursuerTargetPositions,evaderPose(1:2,:)];
pursuerTargetHeadings = [[-1;0]*ones(1,numberTeams),headVec(evaderPose(3,:))];
pursuerTargetSpeeds = [zeros(1,numberTeams),evaderMaxSpeed*ones(1,numEvaders)];
% numFormations = length(formationNames);
% formAngles = zeros(teamSize,teamSize,numFormations);
% formDistances = zeros(teamSize,teamSize,numFormations);

% for kk = 1:length(formationNames)
%     switch formationNames{kk}
%         case '/'
%             % Left Echelon: robots form a line at 45 degrees
%             y = lineSep*([1;1]*[1,0,1]).*headVec([5*pi/4,0,pi/4]);
%         case '\'
%             % Right Echelon: robots form a line at -45 degrees
%             y = lineSep*([1;1]*[1,0,1]).*headVec([7*pi/4,0,3*pi/4]);
%         case '<'
%             % Vee: robots form a "V", first at -45 then +45 degrees
%             y = lineSep*([1;1]*[1,0,1]).*headVec([7*pi/4,0,pi/4]);
%             % formAngles(:,:,kk) = deg2rad(45)*[-1;0;1]*[0,1,0]; % [rad]
%         case '>'
%             % Wedge: robots form a "^", first at +45 then -45 degrees
%             y = lineSep*([1;1]*[1,0,1]).*headVec([5*pi/4,0,3*pi/4]);
%             % formAngles(:,:,kk) = deg2rad(45)*[-3;0;3]*[0,1,0]; % [rad]
%         case '|'
%             % Barrier: robots form a line at 90 degrees
%              y = headVec([3*pi/2,0,pi/2]);
%             % formAngles(:,:,kk) = deg2rad(45)*[-2;0;2]*[0,1,0]; % [rad]
%     end
%     % formAngles(:,:,kk) = atan2(y(2,:).'-y(2,:),y(1,:).'-y(1,:)); % [rad]
%     % formDistances(:,:,kk) = hypot(y(2,:)-y(2,:).',y(1,:)-y(1,:).'); % [m]
% end

% Define the formations as rooted out-branching trees with  leader agent in
% middle and two first followers on either side using a digraph.
AdjacencyROT = [0,1,0;0,0,0;0,1,0];
%% Barrier function handles
[hDomain,dhDomain] = boundaryBarrier(...
    [1,-1,0,0;
    0,0,1,-1],...
    [domainBoundaries(1:2),0,0;
    0,0,domainBoundaries(3:4)] + 1.1*robotRadius*[1,-1,0,0;0,0,1,-1]);


%% Visualize
% Load and initialize floor projection scripts
load('projectiveTransform.mat','H')
hFig = figure(1);
clf
set(1,'Color','w')
linpar = linspace(0,1);
hold on
axis tight equal off
% Construct an extended display object
extDisp = extendedDisplay(H,hFig);
%set(extDisp.hAxesPr,'YDir','reverse','XDir','reverse')
hDom = extDisp.patch(domainBoundaries([1,1,2,2]),domainBoundaries([3,4,4,3]),'LineWidth',4,'EdgeColor','k','FaceColor',[255,251,238]/255);
hPursuerTraj = gobjects(numPursuers,2);
hEvaderTraj = gobjects(numEvaders,2);
hEvaderInit = gobjects(numEvaders,2);
hEvaderInitTraj = gobjects(numEvaders,2);
pursuerColorMat = 0.7*hsv(numPursuers);
evaderColor = [0,0,1];%[0.5,0.1,0.05];
evaderInitColor = [1, 0, 0];


for ii = 1:size(hPursuerTraj,1)
    hPursuerTraj(ii,:) = extDisp.plot(nan,nan,'Color',pursuerColorMat(ii,:),'LineWidth',1);
end
for ii = 1:size(hEvaderTraj,1)
    hEvaderTraj(ii,:) = extDisp.plot(nan,nan,'Color',evaderColor(ii,:),'LineWidth',1);
end

rVisData = kheperaIV_patch(robotRadius);
rVisDataEvader = kheperaIV_patch(robotRadius,evaderColor);
rVisDataEvaderInit = kheperaIV_patch(robotRadius,evaderInitColor);
transform_mat = @(xy,th) [[cos(th-pi/2),-sin(th-pi/2);sin(th-pi/2),cos(th-pi/2)],xy;0,0,1].';
hPursuers = gobjects(numPursuers,2);

for ii = 1:numPursuers
    tempVertices = rVisData.vertices*transform_mat(initPursuerPose(1:2,ii),initPursuerPose(3,ii));
    hPursuers(ii,:) = extDisp.patchFV(rVisData.faces,tempVertices,...
        'FaceVertexCData',rVisData.colors,'FaceColor','flat','EdgeColor','k','LineWidth',0.2);
end
hEvaders = gobjects(numEvaders,2);
for ii = 1:numEvaders
    tempVertices = rVisDataEvader.vertices*transform_mat(initEvaderPose(1:2,ii),initEvaderPose(3,ii));
    tempInitVertices = rVisDataEvaderInit.vertices*transform_mat(initEvaderPose(1:2,ii),initEvaderPose(3,ii));
    hEvaders(ii,:) = extDisp.patchFV(rVisDataEvader.faces,tempVertices,'FaceColor','flat','FaceVertexCData',rVisDataEvader.colors,'EdgeColor','k','LineWidth',0.2);
    hEvaderInit(ii,:) = extDisp.patchFV(rVisDataEvaderInit.faces,tempInitVertices,'FaceColor','flat','FaceVertexCData',rVisDataEvaderInit.colors,'EdgeColor','k','LineWidth',0.2);
end

hGoal = extDisp.patch(goalLocation(1)+0.95*robotRadius*cos(2*pi*linpar(1:end-1)),goalLocation(2)+0.95*robotRadius*sin(2*pi*linpar(1:end-1)),'FaceColor','y');
timerStringPos = [mean(domainBoundaries(1:2)),domainBoundaries(4)];
hTimer = extDisp.text(timerStringPos(1),timerStringPos(2),['t = ',sprintf('%1.2f',0),' s'],'HorizontalAlignment','center','VerticalAlignment','top','FontSize',16);

%% main loop
rng(1);
maxIterations = 1e5;
PURPOS = nan(3,numPursuers,maxIterations);
EVAPOS = nan(3,numEvaders,maxIterations);

TVEC = nan(1,maxIterations);
pursuerPose = initPursuerPose;
evaderPose = initEvaderPose;
barrierGain = 1e0;
controlGain = 2*pi;
maxRotRate = 3*pi;
S = [0,1;-1,0];
newTimer = tic;
prevTime = toc(newTimer);
currentTime = 0;
dt = 0.1;
rng(1,'twister')


for kk = 1:maxIterations
    currentTime = toc(newTimer);
    dt = 0.1; %%currentTime - prevTime;
    prevTime = currentTime;
    tt = currentTime;
    %tt = tt + dt;
    
    PURPOS(:,:,kk) = pursuerPose;
    EVAPOS(:,:,kk) = evaderPose;
    TVEC(kk) = tt;
    % Create a time vector based on step time (dt) and number of steps (kk)
    timeSlice = (0:kk-1) * dt; 

    %Update pursuer targets
    pursuerTargetPositions(:,numberTeams+1:end) = evaderPose(1:2,:);
    pursuerTargetHeadings(:,numberTeams+1:end) = headVec(evaderPose(3,:));

    evaderHeading = [cos(evaderPose(3,:));sin(evaderPose(3,:))];
    goalDir = goalLocation-evaderPose(1:2,:);
    evaderRefDir = goalDir/norm(goalDir);
    switch evaderControl
        case 'Go2Goal'
            % Go to goal
            % evaderRefDir = goalDir;
        case 'stochastic'
            % Stochastic direction
            % Grid size
            stochasticGridSize = 0.5;
            evaderRefDir = stochasticStep(evaderRefDir,evaderMaxSpeed,stochasticGridSize);
    end
    evaderTurnRate = controlGain*evaderHeading.'*S*evaderRefDir;
    % Turn away from pursuers that are too close
    directionVec = evaderPose(1:2,:)-pursuerPose(1:2,:);
    repulsionWeight = 1e2*exp(-vecnorm(directionVec).^2/(2*(2*robotRadius*0.8)^2));
    evaderTurnRate = evaderTurnRate + ...
        controlGain*evaderHeading.'*S*(directionVec./vecnorm(directionVec))*repulsionWeight.';
    % Saturate turn rate if necessary
    if abs(evaderTurnRate) > maxRotRate
        evaderTurnRate = sign(evaderTurnRate)*maxRotRate;
    end
    evaderVel = evaderMaxSpeed*[evaderHeading;0] + evaderTurnRate*[0;0;1];
    dxuEvader =[evaderMaxSpeed; evaderTurnRate];
    pursuerVel = zeros(3,numPursuers);
    dxuPursuer = zeros(2,numPursuers);
     for ii = 1:numberTeams
        if deployTimes(ii) <= currentTime
            teamIDs = sum(groupSizes(1:ii-1))+(1:groupSizes(ii));
            % Pursuer velocity reference
            pursuerHeading = headVec(pursuerPose(3,teamIDs));
            % % Select formation information
            % formationSpec = formDistances(:,:,strcmpi(formationChoice{ii},formationNames)).*AdjacencyROT;
            % formationAngle = formAngles(:,:,strcmpi(formationChoice{ii},formationNames));
            % Compute reference controller
            % [vx,vy] = formationController(pursuerPose(1,teamIDs).',pursuerPose(2,teamIDs).',formationSpec(1:groupSizes(ii),1:groupSizes(ii)),formationAngle(1:groupSizes(ii),1:groupSizes(ii)));
            % pursuerVelRef = [vx,vy].';
            pursuerVelRef = zeros(2, groupSizes(ii));
            % Feedforward team velocity
            if useParallelNav
                pNavPursuerVelRef = pursuerMaxSpeed*parallelNavigation(pursuerPose(1:2,teamIDs(ceil(teamSize/2))),pursuerMaxSpeed,pursuerTargetPositions(:,teamTargets),pursuerTargetSpeeds(teamTargets),pursuerTargetHeadings(:,teamTargets));
                if ~isreal(pNavPursuerVelRef)
                    disp('Parallel Nav failed, using constant speed')
                    pursuerVelRef = pursuerVelRef + 0.75*pursuerMaxSpeed*[1;0];
                else
                    pursuerVelRef = pursuerVelRef + 0.9*pNavPursuerVelRef;
                end
            else
                pursuerVelRef = pursuerVelRef + 0.75*pursuerMaxSpeed*[1;0];
            end
            % Apply the boundary barrier
            pursuerVelRefSafe = barrierControlSI(pursuerPose(1:2,teamIDs),pursuerVelRef,hDomain,dhDomain,barrierGain);
            % if any(abs(pursuerVelRefSafe - pursuerVelRef)>0.01,'all')
            %    disp('mismatch between reference and safe control')
            % end
            pursuerSpeed = sum(pursuerHeading.*pursuerVelRefSafe);
            pursuerTurnRate = controlGain*sum((S.'*pursuerHeading).*pursuerVelRefSafe);
            % Saturate rates if necessary
            pursuerSpeed(abs(pursuerSpeed)>pursuerMaxSpeed) = sign(pursuerSpeed(abs(pursuerSpeed)>pursuerMaxSpeed)).*pursuerMaxSpeed;
            pursuerTurnRate(abs(pursuerTurnRate)>maxRotRate) = sign(pursuerTurnRate(abs(pursuerTurnRate)>maxRotRate))*maxRotRate;
            
            pursuerVel(:,teamIDs) = [...
                pursuerSpeed.*pursuerHeading(1,:);
                pursuerSpeed.*pursuerHeading(2,:);
                zeros(size(pursuerSpeed))] + [0;0;1]*pursuerTurnRate;
            dxuPursuer(:,teamIDs) = [pursuerSpeed; pursuerTurnRate];
        end
        end

    % Integrate
    pursuerPose = pursuerPose + dt*pursuerVel;
    evaderPose = evaderPose + dt*evaderVel;
    % Wrap angles
    pursuerPose(3,:) = atan2(sin(pursuerPose(3,:)),cos(pursuerPose(3,:)));
    evaderPose(3,:) = atan2(sin(evaderPose(3,:)),cos(evaderPose(3,:)));

    for ii = 1:numPursuers
        tempVertices = rVisData.vertices*transform_mat(pursuerPose(1:2,ii),pursuerPose(3,ii));
        extDisp.setFV(hPursuers(ii,:),tempVertices);
        extDisp.set(hPursuerTraj(ii,:),squeeze(PURPOS(1,ii,:)),squeeze(PURPOS(2,ii,:)))
    end
    
    for ii = 1:numEvaders
        tempVertices = rVisDataEvader.vertices*transform_mat(evaderPose(1:2,ii),evaderPose(3,ii));
        extDisp.setFV(hEvaders(ii,:),tempVertices);
        extDisp.set(hEvaderTraj(ii,:),squeeze(EVAPOS(1,ii,:)),squeeze(EVAPOS(2,ii,:)))
    end
    
    extDisp.setText(hTimer,timerStringPos(1),timerStringPos(2),'String',['t = ',sprintf('%1.2f',currentTime),' s'])
    
    drawnow limitrate

        %%%
    % Terminating conditions:
    %
    % Domain Escape
    if ~all(inpolygon([pursuerPose(1,:),evaderPose(1,:)],[pursuerPose(2,:),evaderPose(2,:)],domainBoundaries([1,1,2,2])+robotRadius*[1,1,-1,-1],domainBoundaries([3,4,4,3])+robotRadius*[1,-1,-1,1]))
        finalStatusFlag = 1;
        warning('Some of the robots have left the domain. Terminating...')
        break
    end
    %%%
    % Pursuer-Evader Collision
    if any(hypot(pursuerPose(1,:)-evaderPose(1,:).',pursuerPose(2,:)-evaderPose(2,:).')<1.05*2*robotRadius,'all')
        finalStatusFlag = 2;
        disp('At least one pursuer has captured the evader!')
%         disp(hypot(pursuerPose(1,:)-evaderPose(1,:).',pursuerPose(2,:)-evaderPose(2,:).'))
%         disp('velocity')
%         disp(dxuEvader)
        break
    end
    %%%
    % Evader Reached Goal
    if any(hypot(goalLocation(1,:)-evaderPose(1,:).',goalLocation(2,:)-evaderPose(2,:).')<0.95*robotRadius,'all')
        finalStatusFlag = 0;
        disp('The evader has reached the goal!')
        break
    end
end
%%

%% Auxiliary functions
% Formation controller reference
% function [vx,vy] = formationController(x,y,formSpec,formAngle)
% % formationController returns velocity reference to achieve desired
% % formation using a directed formation graph. Graph is assumed to be a
% % rooted out-branching tree with leader node as root. The reference
% % velocity is given by an extended consensus controller
% %
% % $$v = \sum_{j\in\cal N} (w_{ij}\,I+r_{ij}\,S)\,(x_j-x_i)$$
% %
% % where the scalar weights wij and rij  regulate distance and angle, resp.
% %
% % Inputs x,y define the x and y state coordinates of the robots at time t.
% %
% % formSpec is a square matrix containing the structure of the formation
% % specification. A zero entry suggests agents do not need to maintain a
% % distance specification. A nonzero entry encodes the desired distance
% % specification. The matrix need not be symmetric when the specification
% % encodes a digraph topology.
% %
% % formAngle is a square matrix containing the angle at which agents must
% % maintain formation relative to neighbors.
% %
% % Internal control gains
% distGain = 5e0;
% rotGain = 5e0;
% % Reshape inputs
% x = x(:);
% y = y(:);
% % Get ajacency matrix
% AdjMat = (formSpec~=0);
% % Compute distance between agents. Entry ij contains ||x_i-x_j||.
% distMat = hypot((x-x.'),(y-y.'));
% % Compute the formation distance regulation weight
% wMat = distGain*((distMat - formSpec)./(distMat+eye(size(distMat)))).*AdjMat;
% % Compute angle between agents. Entry ij contains ?(x_j-x_i) (from i to j).
% angleMat = atan2((y-y.'),(x-x.'));
% % Compute the formation angle regulation weight
% rotAng = rotGain*sin(angleMat-formAngle).^3./(distMat+eye(size(distMat))).*AdjMat;
% % Compute matrix weight elements (Symmetric and Skew-symmetric parts)
% wMatXX =  wMat;
% wMatYY =  wMat;
% wMatXY = -rotAng;
% wMatYX =  rotAng;
% % Compute the extended consensus protocol
% vx = (wMatXX*x-sum(wMatXX,2).*x)+(wMatXY*y-sum(wMatXY,2).*y);
% vy = (wMatYX*x-sum(wMatYX,2).*x)+(wMatYY*y-sum(wMatYY,2).*y);
% end
% Barrier control function for single integrator systems
function [v] = barrierControlSI(x,uRef,barFun,dBarFundx,barrierGain)
% barrierControl returns the min-norm solution to the quadratic program
%
% v* = arg min_u ||u-uRef||^2
%
% subject to p constraints d(BarrierFunc(x,u))/dt >=
% -barrierGain*barrierFunc(x) for a single integrator system of the form
% dx/dt = u. Practically, the greater the value of barrierGain, the closer
% the system can approach the barrier. The closed-form solution depends on
% the sign of the function:
%
% F = dBarFundx*uRef + barrierGain*barFun
%
% which determines whether the dynamic constraint if violated. The min-norm
% solution is v = uRef if F >= 0, i.e., if the constraint is not violated.
% Otherwise, the projection of F onto dBdx must be subtracted from the
% reference to ensure that the constraint will be satisfied and the safe
% set is forward invariant.
%%%
% Evaluate the barrier and its derivative, and use them to compute F.
h = barFun(x); % p-by-N matrix for p values of h for each of the N agents in x
dhdx = dBarFundx(x); % p-by-N-by-d matrix of p values of dhdx for N agents with d states each
F = sum(bsxfun(@times,dhdx,permute(uRef,[3,2,1])),3) + barrierGain*h.^3; % p-by-N of values of F for each x
v = uRef;
% Find agents whose reference must be modified for each boundary
for kk = 1:size(h,1)
    ii =  F(kk,:) < 0 ;
    if any(ii)
        dhdxi = dhdx(kk,ii,:);
        % Subtract from the reference signal the projection of the
        % components that violate the constraints.
        v(:,ii) = v(:,ii) - bsxfun(@times,F(kk,ii)./dot(dhdxi,dhdxi,3),ipermute(dhdxi,[3,2,1]));
    end
end
end
% Barrier function for boundary collisions
function [h, dhdx] = boundaryBarrier(n,p,varargin)
% hyperplaneBarrier returns the function handles for a barrier function and
% its state derivative for a convex polytope boundary. The hyperplane for
% each of the m boundaries are defined by points on the planes p =
% [p1,p2,...,pm] and unit normals n = [n1,n2,...,nm] pointing towards the
% safe set. The size of the multi-agent system can be provided as a third
% argument to reduce the number of operations needed in each handle.
%
% n and p are assumed to be d-by-m vectorss. Output handles accept d-by-N
% collection of states for N agents as inputs and return m-by-N values for
% first argument, and m-by-n-by-d for second argument.
b = dot(n,p,1).'; % m-by-1 vector of m dot products
w = n.'; % m-by-d vector of m row vector normals
if nargin == 2 % Expand based on number of columns in input argument x
    h = @(x) bsxfun(@minus,w*x,b);
    dhdx = @(x) repmat(permute(w,[1,3,2]),1,size(x,2));
else % Pre-expand matrices, independent of input argunment x
    temp1 = repmat(b,1,varargin{1});
    h = @(x) w*x-temp1;
    temp2 = repmat(permute(w,[1,3,2]),1,varargin{1});
    dhdx = @(x) temp2;
end
end
% Stochastic evader controller
function [stochasticDirection] = stochasticStep(genDir,speed,varargin)
%stochasticStep Introduces randomness in the motion of an agent given a
%general direction genDir and the speed speed of the agent. The motion is
%discretized to a grid. The grid size in meters can be provided as a third
%argument, otherwise a default size of 0.1 meters is assumed.
if nargin > 2
    gridSize = varargin{1};
else
    gridSize = 0.1;
end
% Select a leaf position according to the binomial distribution limit
% This is also the vertical step
verStep = binornd( round(speed/gridSize), abs( genDir(2) ), [1 1] );
% Calc the horizontal step
horStep = speed/gridSize - verStep;

% Get the signs right
stochasticDirection = sign(genDir).*[horStep;verStep]*gridSize;

end
% Parallel Navigation
function [up] = parallelNavigation(pp,sp,ee,se,ue)
%stochasticparallelnavemulate The continuous Euclidean version of parallel
%nav
%   Input(s):
%       pp = pursuer position (2x1)
%       sp = pursuer speed (1x1)
%       ee = evader position (2x1)
%       se = evader speed (1x1)
%       ue = evader direction/heading/unit-vector (2x1)
%
%   Output(s):
%       up = pursuer direction/heading/unit-vector reference/command (2x1)

% First we need to find the direction of the line of sight...error
% direction
ll = (pp-ee);

% Normalize/versor
ul = ( 1/norm(ll) )*ll;

% Need the closing rate
aa = 1;
bb = -2*se*dot(ue,ul,1);
cc = -(sp^2-se^2);

sl = (1/2/aa)*( -bb + sqrt(bb^2-4*aa*cc) );

% Calculate the pursuer direction according to parallel nav
up = (1/sp)*(se*ue-sl*ul);

end