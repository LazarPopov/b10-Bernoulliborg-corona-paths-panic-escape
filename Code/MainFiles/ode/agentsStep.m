function [ simulationObj ] = agentsStep( simulationObj, settings, hObject)
%AGENTSSTEP calculates the new agent matrix after one time step with ode23
%   and the right hand side of the ODE of this model defined by odeRhs or
%   odeRhsWithPressure if pressure calculations are included
% see also: ODE23
agents = simulationObj.agents;
t = simulationObj.tSimulation;
columns = simulationObj.columns;
wallLines = simulationObj.wallLines;

dt = settings.dtPlot; %get 'dt'
pressureBool = settings.pressureBool;

NAgent = size(agents, 1); %get number of agents
NExits = size(simulationObj.exitCoord,1);

%%---ode23 integration-----------------------------------------------------
%This will be changed with closest agent function where it separres agent to different sets and each set of agents
%goes to the closest exit
if NExits == 2
  half=NAgent/2;
  agents1 = agents(1:half,:);
  agents2 = agents(half + 1 : 50,:);
  NAgent1 = size(agents1, 1);
  NAgent2 = size(agents2, 1);
  testArray = zeros(NExits,NAgent1,5);
  testArray(1, :, :) = agents1;
  testArray(2, :, :) = agents2;
elseif  NExits == 5
  fifth=NAgent/5;
  agents1 = agents(1:fifth,:);
  agents2 = agents(fifth + 1 : 2*fifth,:);
  agents3 = agents(2*fifth + 1 : 3*fifth,:);
  agents4 = agents(3*fifth + 1 : 4*fifth,:);
  agents5 = agents(4*fifth + 1 : 5*fifth,:);
  NAgent1 = size(agents1, 1);


  testArray = zeros(NExits,NAgent1,5);
  testArray(1, :, :) = agents1;
  testArray(2, :, :) = agents2;
  testArray(3, :, :) = agents3;
  testArray(4, :, :) = agents4;
  testArray(5, :, :) = agents5;


else
    testArray(1,:,:) = agents;
end

  finalAgents = [];
  for j = 1:NExits
      currentExit = simulationObj.exitCoord( j,:);
      closestAgents = testArray(j, :, :);
      NClosestAgents =  size(closestAgents,2);

      odeVec = reshape(closestAgents(1,:,1:4),4*NClosestAgents,1); %create 'odeVec' initial state column vector (with radius)

      radii = transpose(closestAgents(1,:,5));
      odeOptions = odeset('AbsTol',1e-2,'RelTol',1e-2, 'Events',@(t,y) odeEventFunction(t,y,currentExit)); % RelTol: measure of error relative to size of each solution component. Default is 1e-3
      if pressureBool
          [tVec, odeAgents, TE, ~, IE] = ode23(@(t,y)odeRhsWithPressure(t,y,radii,columns,wallLines,currentExit,settings, hObject),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
          handles = guidata(hObject);
          if j == 1
            simulationObj.pressure = handles.simulationObj.pressure;
          else
            simulationObj.pressure = [simulationObj.pressure;  handles.simulationObj.pressure];
          end
      else
          [tVec, odeAgents, TE, ~, IE] = ode23(@(t,y)odeRhs(t,y,radii,columns,wallLines,currentExit,settings),[t,t+dt],odeVec,odeOptions); %solve ODE with 'ode23'
    end

    closestAgents(1,:,1:4) = reshape(odeAgents(end,:),NClosestAgents,4); %uptate 'agents' matrix

    %the 3 statements below have to be tested and fixed
    timesAgentsThroughDoor = TE(IE <= NClosestAgents);
    allThroughDoor = any(IE == NClosestAgents + 1);
    simulationObj.tSimulation = tVec(end);


    finalAgents = [finalAgents closestAgents];

  end

  finalAgents = squeeze(finalAgents);
  %the 3 statements below have to be tested and fixed
  simulationObj.agents = finalAgents;
  simulationObj.allThroughDoor = allThroughDoor;
  simulationObj.timesAgentsThroughDoor = [simulationObj.timesAgentsThroughDoor; timesAgentsThroughDoor];




%toc
end
