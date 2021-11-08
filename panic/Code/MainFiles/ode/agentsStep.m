function [ simulationObj ] = agentsStep( simulationObj, settings, hObject)

function [mp] = midpoint(arrayWith4Columns)
  x1 = arrayWith4Columns(:, 1);
  y1 = arrayWith4Columns(:, 2);
  x2 = arrayWith4Columns(:, 3);
  y2 = arrayWith4Columns(:, 4);
  mp = [x1+x2, y1+y2]/2;
  return
end
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

agentsExitsMatrix = zeros(NExits,size(agents, 1) ,5);
exitCoordMidPoints = midpoint(simulationObj.exitCoord);
indexesOfClosestExit=dsearchn(exitCoordMidPoints(:,:), agents(:, 1:2) );

for i=1:NAgent
  agentsExitsMatrix(indexesOfClosestExit(i),i,:) = agents(i,:);
end

  finalAgents = [];

  for j = 1:NExits

      currentExit = simulationObj.exitCoord( j,:);

      % closesest agents to exit j with 0s in the array
      closestAgents = agentsExitsMatrix(j, :, :); %testArray(j, :, :);


      NClosestAgents =  size(closestAgents,2);

      % buffer to remove the zeros
      bufferClosesetAgents = zeros(NClosestAgents,5);
      bufferClosesetAgents = squeeze(closestAgents(1,:,:));
      bufferClosesetAgents( all(~bufferClosesetAgents,2), : ) = [];
      bufferSize= size(bufferClosesetAgents,1);

      % change the size of the closeset agents and save the values from the buffer values.
      closestAgents=zeros(1,bufferSize,5);
      closestAgents(1, :, :) = bufferClosesetAgents;
      NClosestAgents = bufferSize;

      if NClosestAgents == 0
        continue
      end


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
