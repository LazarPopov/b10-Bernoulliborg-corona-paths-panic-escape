function newExitLineButtonUp( src, evnt )
%NEWEXITLINEBUTTONUP stores new exit data
thisfig = gcf();
set(thisfig,'WindowButtonUpFcn','');
set(thisfig,'WindowButtonMotionFcn','');
handles = guidata(thisfig);

% figure out start and end point of line
startPoint = handles.temp.startPoint;
endPoint = get(gca,'CurrentPoint');
endPoint = endPoint(1,[1,2]);

newExitLine =  [startPoint, endPoint];

handles.simulationObj.exitCoord = [handles.simulationObj.exitCoord; newExitLine];

disp('handles.simulationObj.exitCoord')
disp(handles.simulationObj.exitCoord)
disp('handles.simulationObj.exitCoord(1, :)')
disp(handles.simulationObj.exitCoord(1,:))

% store wall id in wall drawings
hNewExitLine = handles.temp.hExit;
handles.plotObj.hExit = [handles.plotObj.hExit,hNewExitLine];

set(handles.temp.hExit(1), 'UserData', [4, handles.temp.hExit(2)]);
set(handles.temp.hExit(2), 'UserData', [4, handles.temp.hExit(1)]);

% tidy up
handles = rmfield(handles, 'temp');

% Update handles structure
guidata(thisfig, handles);
end
