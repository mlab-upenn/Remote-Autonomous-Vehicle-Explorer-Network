function retStruct = getRF(serialID, handles)
retStruct = handles;

% definitions
START_DELIM = 126;
RX_PACKET_T = 144;

% if there is nothing to read, do nothing and return
if(serialID.BytesAvailable == 0)
    retStruct = handles;
    return
end

% otherwise we scan for a start delimeter
while(serialID.BytesAvailable > 0)
    thisByte = fread(serialID,1);
    if thisByte == START_DELIM
        % read lengthH, lengthL, and discard
        fread(serialID,2);
        packetT = fread(serialID,1);
        if packetT == RX_PACKET_T
            break;
        end
    end
end

% if we got here we found a receive packet, read all bytes in
% first read bytes we don't care about
fread(serialID,11);

% now parse the payload
unitIdentifier = fread(serialID,1);

yaw = (360*fread(serialID,1)/255);
pitch = (360*fread(serialID,1)/255);
roll = (360*fread(serialID,1)/255);
vX = fread(serialID,1);
vY = fread(serialID,1);
vZ = fread(serialID,1);
alt = fread(serialID,2);
alt = hex2dec([dec2hex(alt(1)) dec2hex(alt(2))]);
bat = fread(serialID,1)/10;
IR = zeros(1,4);
IR(1) = fread(serialID,1);
IR(2) = fread(serialID,1);
IR(3) = fread(serialID,1);
IR(4) = fread(serialID,1);

switch unitIdentifier
    case 1
        % leader
        retStruct.lead_yaw = yaw;
        retStruct.lead_pitch = pitch;
        retStruct.lead_roll = roll;
        retStruct.lead_vX = vX;
        retStruct.lead_vY = vY;
        retStruct.lead_vZ = vZ;
        retStruct.lead_alt = alt;
        retStruct.lead_bat = bat;
        retStruct.lead_IR = IR;
    case 2
        % follower
        retStruct.fol1_yaw = yaw;
        retStruct.fol1_pitch = pitch;
        retStruct.fol1_roll = roll;
        retStruct.fol1_vX = vX;
        retStruct.fol1_vY = vY;
        retStruct.fol1_vZ = vZ;
        retStruct.fol1_alt = alt;
        retStruct.fol1_bat = bat;
        retStruct.fol1_IR = IR;
    otherwise
        % do nothing
end

end