function [ simulationObj ] = agentsStep( simulationObj, settings, hObject)
%AGENTSSTEP calculates the new agent matrix after one time step with ode23
%   and the right hand side of the ODE of this model defined by odeRhs or
%   odeRhsWithPressure if pressure calculations are included
% see also: ODE23

disp('test0')
agents = simulationObj.agents;
t = simulationObj.tSimulation;
columns = simulationObj.columns;
wallLines = simulationObj.wallLines;
% remains from random choice exit
% c = clock
% rng(1,'twister');
% if size(simulationObj.exitCoord,1) > 1
%   r = randi([1, size(simulationObj.exitCoord,1)],1);
%   exitCoord = simulationObj.exitCoord( r,:)
% else
%   exitCoord = simulationObj.exitCoord
% end
dt = settings.dtPlot; %get 'dt'
pressureBool = settings.pressureBool;

NAgent = size(agents, 1); %get number of agents
NExits = size(simulationObj.exitCoord,1)

%%---ode23 integration-----------------------------------------------------



%%%%%%%%%%%% TRYING TO MAKE THE CHOICE WITH A FOR LOOP FOR ARBITRARY NUMBER OF EXITS

%Having 2 exits chosen by different agents
% testGrid = ndgrid(agents1,agents2)
%
% disp(testGrid)
% %
% disp('size(testGrid)')
% disp(size(testGrid))
% %the seperation should be made in the for loop at each iteration
% %closest agents to the jth exit
% %therefore we need a function that determines the agent
% for j = 1:NExits
%     currentExit = simulationObj.exitCoord( j,:);
%     closestAgents = 0;
%     NClosestAgents =  size(closestAgents);
%
%     odeVec = reshape(closestAgents(:,1:4),4*NClosestAgents,1); %create 'odeVec' initial state column vector (with radius)
%     radii = closestAgents(:,5);
%     odeOptions = odeset('AbsTol',1e-2,'RelTol',1e-2, 'Events',@(t,y) odeEventFunction(t,y,currentExit)); % RelTol: measure of error relative to size of each solution component. Default is 1e-3
%     disp('test5')
%     if pressureBool
%         [tVec, odeAgents, TE, ~, IE] = ode23(@(t,y)odeRhsWithPressure(t,y,radii,columns,wallLines,currentExit,settings, hObject),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
%         handles = guidata(hObject);
%         simulationObj.pressure = handles.simulationObj.pressure;
%     else
%         disp('5.1');
%         [tVec, odeAgents, TE, ~, IE] = ode23(@(t,y)odeRhs(t,y,radii,columns,wallLines,currentExit,settings),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
%         disp('TE1')
%         disp(TE)
%         disp('IE1')
%         disp(IE)
%   end
% end
% agents1(:,1:4) = reshape(odeAgents(end,:),NAgent1,4); %uptate 'agents' matrix
% agents2(:,1:4) = reshape(odeAgents2(end,:),NAgent2,4); %uptate 'agents' matrix
% timesAgentsThroughDoor = TE(IE <= NAgent1);
% allThroughDoor = any(IE == NAgent1 + 1);
% simulationObj.tSimulation = tVec(end);
% disp('test7')
% size(agents1)
% size(agents2)
% simulationObj.agents = cat(1,agents1,agents2);
% disp(simulationObj.agents)
% simulationObj.allThroughDoor = allThroughDoor;
% disp('test8')
% simulationObj.timesAgentsThroughDoor = [simulationObj.timesAgentsThroughDoor; timesAgentsThroughDoor];


%%%%% IF THERE IS MORE THAN 1 EXITS
if size(simulationObj.exitCoord,1) > 1
  half=NAgent/2

  disp('test1')

  agents1 = agents(1:half,:);
  disp('test2')
  agents2 = agents(half + 1 : 50,:);
  disp('test3')

  NAgent1 = size(agents1, 1);
  NAgent2 = size(agents2, 1);


  exitCoord1 = simulationObj.exitCoord( 1,:)
  exitCoord2 = simulationObj.exitCoord( 2,:)

  disp('test4')

  odeVec = reshape(agents1(:,1:4),4*NAgent1,1); %create 'odeVec' initial state column vector (with radius)
  radii = agents1(:,5);
  odeOptions = odeset('AbsTol',1e-2,'RelTol',1e-2, 'Events',@(t,y) odeEventFunction(t,y,exitCoord1)); % RelTol: measure of error relative to size of each solution component. Default is 1e-3
  disp('test5')
  if pressureBool
      [tVec, odeAgents, TE, ~, IE] = ode23(@(t,y)odeRhsWithPressure(t,y,radii,columns,wallLines,exitCoord1,settings, hObject),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
      handles = guidata(hObject);
      simulationObj.pressure = handles.simulationObj.pressure;

  else
      disp('5.1');
      [tVec, odeAgents, TE, ~, IE] = ode23(@(t,y)odeRhs(t,y,radii,columns,wallLines,exitCoord1,settings),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
      disp('TE1')
      disp(TE)
      disp('IE1')
      disp(IE)
  end


  odeVec = reshape(agents2(:,1:4),4*NAgent2,1); %create 'odeVec' initial state column vector (with radius)
  radii = agents2(:,5);
  odeOptions = odeset('AbsTol',1e-2,'RelTol',1e-2, 'Events',@(t,y) odeEventFunction(t,y,exitCoord2)); % RelTol: measure of error relative to size of each solution component. Default is 1e-3
  disp('test5.12')
  if pressureBool
      [tVec, odeAgents2, TE, ~, IE] = ode23(@(t,y)odeRhsWithPressure(t,y,radii,columns,wallLines,exitCoord2,settings, hObject),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
      handles = guidata(hObject);
      simulationObj.pressure = [simulationObj.pressure;  handles.simulationObj.pressure];
      disp('5.2');
  else
      [tVec, odeAgents2, TE, ~, IE] = ode23(@(t,y)odeRhs(t,y,radii,columns,wallLines,exitCoord2,settings),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
  end
  agents1(:,1:4) = reshape(odeAgents(end,:),NAgent1,4); %uptate 'agents' matrix
  agents2(:,1:4) = reshape(odeAgents2(end,:),NAgent2,4); %uptate 'agents' matrix
  timesAgentsThroughDoor = TE(IE <= NAgent1);
  allThroughDoor = any(IE == NAgent1 + 1);
  simulationObj.tSimulation = tVec(end);
  disp('test7')
  size(agents1)
  size(agents2)
  simulationObj.agents = cat(1,agents1,agents2);
  disp(simulationObj.agents)
  simulationObj.allThroughDoor = allThroughDoor;
  disp('test8')
  simulationObj.timesAgentsThroughDoor = [simulationObj.timesAgentsThroughDoor; timesAgentsThroughDoor];

%IF THERE IS 1 EXIT
else
  disp('test9')
  exitCoord = simulationObj.exitCoord
  odeVec = reshape(agents(:,1:4),4*NAgent,1); %create 'odeVec' initial state column vector (with radius)
  radii = agents(:,5);
  odeOptions = odeset('AbsTol',1e-2,'RelTol',1e-2, 'Events',@(t,y) odeEventFunction(t,y,exitCoord)); % RelTol: measure of error relative to size of each solution component. Default is 1e-3
  if pressureBool
      [tVec, odeAgents, TE, ~, IE] = ode23(@(t,y)odeRhsWithPressure(t,y,radii,columns,wallLines,exitCoord,settings, hObject),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
      handles = guidata(hObject);
      simulationObj.pressure = handles.simulationObj.pressure;
  else
      [tVec, odeAgents, TE, ~, IE] = ode23(@(t,y)odeRhs(t,y,radii,columns,wallLines,exitCoord,settings),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
  end

  agents(:,1:4) = reshape(odeAgents(end,:),NAgent,4); %uptate 'agents' matrix
  timesAgentsThroughDoor = TE(IE <= NAgent);
  allThroughDoor = any(IE == NAgent + 1);
  simulationObj.tSimulation = tVec(end);
  simulationObj.agents = agents;
  simulationObj.allThroughDoor = allThroughDoor;
  simulationObj.timesAgentsThroughDoor = [simulationObj.timesAgentsThroughDoor; timesAgentsThroughDoor];
end
disp('test10')
%toc
end
