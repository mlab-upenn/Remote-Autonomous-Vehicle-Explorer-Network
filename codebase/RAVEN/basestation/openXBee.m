function sID = openXBee()
ports = instrhwinfo('serial');
if length(ports.AvailableSerialPorts) < 1
    error('no serial ports available');
else
    PORT = ports.AvailableSerialPorts{1};
end

%% Opening first available serial port
sXbee = serial(PORT);
set(sXbee,'BaudRate',115200);
set(sXbee,'Timeout',3);
set(sXbee,'Terminator',13);
sXbee.ReadAsyncMode = 'continuous';

sID = sXbee;
end