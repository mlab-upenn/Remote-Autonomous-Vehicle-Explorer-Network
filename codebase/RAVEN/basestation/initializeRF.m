
%% Definitions
BASE_ID = 0;
LEAD_ID = 1;
FOL1_ID = 2;

%% Information handle
handles.lead_yaw = 0;
handles.lead_pitch = 0;
handles.lead_roll = 0;
handles.lead_vX = 0;
handles.lead_vY = 0;
handles.lead_vZ = 0;
handles.lead_alt = 0;
handles.lead_bat = 0;
handles.lead_IR = [0 0 0 0];

handles.fol1_yaw = 0;
handles.fol1_pitch = 0;
handles.fol1_roll = 0;
handles.fol1_vX = 0;
handles.fol1_vY = 0;
handles.fol1_vZ = 0;
handles.fol1_alt = 0;
handles.fol1_bat = 0;
handles.fol1_IR = [0 0 0 0];

%% Open XBee serial
serialID = openXBee();
fopen(serialID);

%% Sending test
% create packet
packet = mxRFMakePacket(0,66,1);
while(1)
    %pause();
    %packet = mxRFMakePacket(FOL1_ID,66,1);
    %fprintf('sending...');
    %sendRF(serialID,packet);
    %fprintf('DONE\n');
    pause(0.01);
    getRF(serialID,handles);
    disp(handles);
end

%% Closing Serial Port
fclose(serialID);
