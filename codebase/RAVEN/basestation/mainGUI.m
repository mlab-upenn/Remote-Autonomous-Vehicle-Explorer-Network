function varargout = mainGUI(varargin)
% MAINGUI M-file for mainGUI.fig
%      MAINGUI, by itself, creates a new MAINGUI or raises the existing
%      singleton*.
%
%      H = MAINGUI returns the handle to a new MAINGUI or the handle to
%      the existing singleton*.
%
%      MAINGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAINGUI.M with the given input arguments.
%
%      MAINGUI('Property','Value',...) creates a new MAINGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mainGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mainGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mainGUI

% Last Modified by GUIDE v2.5 13-Jan-2011 17:29:52

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mainGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @mainGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before mainGUI is made visible.
function mainGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mainGUI (see VARARGIN)

% Choose default command line output for mainGUI
handles.output = hObject;
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

handles.LEAD_ID = 1;
handles.FOL1_ID = 2;

handles.serialID = 0;
handles.RFLoop = 1;
handles.serialIsOpen = 0;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mainGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mainGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in butInitRF.
function butInitRF_Callback(hObject, eventdata, handles)
% hObject    handle to butInitRF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% set BG to orange to show we're working
set(hObject,'BackgroundColor',[1 0.85 0]);
pause(0.25);

%% Definitions
BASE_ID = 0;
LEAD_ID = 1;
FOL1_ID = 2;

%% Open XBee serial
if(handles.serialIsOpen == 0)
    handles.serialID = openXBee();
    fopen(handles.serialID);
    handles.serialIsOpen = 1;
    handles.RFLoop = 1;
    set(hObject,'BackgroundColor',[0.5 1 0.5]);
end

% Update handles structure
guidata(hObject, handles);

%% Receive Data

if handles.serialID.BytesAvailable > 0
    fread(handles.serialID,handles.serialID.BytesAvailable);
end
while(handles.RFLoop == 1)
    pause(0.01);
    handles = getRF(handles.serialID,handles);
    if handles.serialID.BytesAvailable > 0
        fread(handles.serialID,handles.serialID.BytesAvailable);
    end
    disp('--ypr--');
    disp(handles.lead_yaw);
    disp(handles.lead_pitch);
    disp(handles.lead_roll);
    % Update handles structure
    guidata(hObject, handles);
    plotData(handles);
end

set(hObject,'BackgroundColor',[1 0.85 0]);


% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in butAutoLand.
function butAutoLand_Callback(hObject, eventdata, handles)
% hObject    handle to butAutoLand (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.serialIsOpen == 1
    packetF = mxRFMakePacket(handles.FOL1_ID,110,1);
    %sendRF(handles.serialID,packetF);
    packetL = mxRFMakePacket(handles.LEAD_ID,110,1);
    sendRF(handles.serialID,packetL);
    
    % repeat to be sure packets were sent
    %sendRF(handles.serialID,packetF);
    sendRF(handles.serialID,packetL);
end



% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in butSpinup.
function butSpinup_Callback(hObject, eventdata, handles)
% hObject    handle to butSpinup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.serialIsOpen == 1
    packetF = mxRFMakePacket(handles.FOL1_ID,120,1);
    %sendRF(handles.serialID,packetF);
    packetL = mxRFMakePacket(handles.LEAD_ID,120,1);
    sendRF(handles.serialID,packetL);
    % repeat to be sure packets were sent
    %sendRF(handles.serialID,packetF);
    sendRF(handles.serialID,packetL);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in butCloseSerial.
function butCloseSerial_Callback(hObject, eventdata, handles)
% hObject    handle to butCloseSerial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.serialIsOpen == 1
    fclose(handles.serialID);
    handles.serialIsOpen = 0;
    handles.RFLoop = 0;
    set(handles.butInitRF,'BackgroundColor',[1 0.8 0.8]);
end

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in butEmergencyStop.
function butEmergencyStop_Callback(hObject, eventdata, handles)
% hObject    handle to butEmergencyStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.serialIsOpen == 1
    disp('Sending Emergency Stop');
    packetF = mxRFMakePacket(handles.FOL1_ID,100,1);
    %sendRF(handles.serialID,packetF);
    packetL = mxRFMakePacket(handles.LEAD_ID,100,1);
    fprintf('Lead 1...');
    sendRF(handles.serialID,packetL);
    fprintf('DONE\n');
    
    % repeat to be sure packets were sent
    %sendRF(handles.serialID,packetF);
    fprintf('Lead 2...');
    sendRF(handles.serialID,packetL);
    fprintf('DONE\n');

end

% Update handles structure
guidata(hObject, handles);
