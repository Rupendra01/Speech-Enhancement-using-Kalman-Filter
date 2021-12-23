function varargout = threat(varargin)
% PR_ROU_GUI MATLAB code for pr_rou_gui.fig
%      PR_ROU_GUI, by itself, creates a new PR_ROU_GUI or raises the existing
%      singleton*.
%
%      H = PR_ROU_GUI returns the handle to a new PR_ROU_GUI or the handle to
%      the existing singleton*.
%
%      PR_ROU_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PR_ROU_GUI.M with the given input arguments.
%
%      PR_ROU_GUI('Property','Value',...) creates a new PR_ROU_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before pr_rou_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to pr_rou_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help pr_rou_gui

% Last Modified by GUIDE v2.5 01-Dec-2021 14:32:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @pr_rou_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @pr_rou_gui_OutputFcn, ...
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


% --- Executes just before pr_rou_gui is made visible.
function pr_rou_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to pr_rou_gui (see VARARGIN)

% Choose default command line output for pr_rou_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
% create an axes that spans the whole gui
ah = axes('unit', 'normalized', 'position', [0 0 1 1]); 
% import the background image and show it on the axes
bg = imread('download2.jpg');
imagesc(bg);
% prevent plotting over the background and turn the axis off
set(ah,'handlevisibility','off','visible','off')
% making sure the background is behind all the other uicontrols
uistack(ah, 'bottom');
% UIWAIT makes pr_rou_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = pr_rou_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and userta (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in openthreatfile.
% function openthreatfile_Callback(hObject, eventdata, handles)
% % hObject    handle to openthreatfile (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% global Input Fs
% % [filename pathname] = uigetfile({'*.wav'}, 'Select File');
% % fullfilename = strcat (pathname, filename);%[Input, Fs] = audioread('input.wav');
% [Input,Fs] = audioread(filename);
% im=imread('project_image.jpg');
% axes(handles.axes1);
% imshow(im);
% f = msgbox('Audio file succesfully loaded','Succes');
% set(f, 'position', [463 471 180 70]);



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Input Fs player
Input = Input(:,1);
Noise = normrnd(0,sqrt(0.001),size(Input));
NoisyInput = Input + Noise;
Time = (0:1/Fs:(length(Input)-1)/Fs)';
player = audioplayer(Input, Fs);
%sound(NoisyInput,Fs);
play(player);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global player
stop(player)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global player
pause(player)

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global player
resume(player)



% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3
global Input Fs
contents = cellstr(get(hObject,'String'));
popupmemu3 = contents(get(hObject,'value'));
if (strcmp(popupmemu3,'0.5X'))
    player = audioplayer(Input, 0.75*Fs);
    play(player);
elseif (strcmp(popupmemu3,'1X'))
    player = audioplayer(Input, Fs);
    play(player);
elseif (strcmp(popupmemu3,'1.5X'))
    player = audioplayer(Input, 1.5*Fs);
    play(player);
elseif (strcmp(popupmemu3,'2X'))
    player = audioplayer(Input, 2*Fs);
    play(player);
end
assignin('base','player',player);

% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in pushbutton7.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%global Output

WinLenSec = 0.0025; % Window length in seconds.
HopPercent = 1; % percentage of hopping.
AROrder = 20; % Auto regressive filter order.
NumIter = 7;

[Input, Fs] = audioread('threat_audio.wav');
Input = Input(:,1);
Noise = normrnd(0,sqrt(0.01),size(Input));
NoisyInput = Input+ Noise;
Time = (0:1/Fs:(length(Input)-1)/Fs)';
%sound(NoisyInput,Fs);


WinLenSamples = fix(WinLenSec * Fs);
Window = ones(WinLenSamples,1);
[ChoppedSignal, NumSegments] = Chopper(NoisyInput, WinLenSamples, Window, HopPercent);


H = [zeros(1,AROrder-1),1];   % Measurement matrix.
R = var(Noise);     % Variance of noise.

[FiltCoeff, Q] = lpc(ChoppedSignal, AROrder);   % Finding filter coefficients.

P = R * eye(AROrder,AROrder);   % Error covariance matrix.
Output = zeros(1,size(NoisyInput,1));   % Allocating memory for output signal.
Output(1:AROrder) = NoisyInput(1:AROrder,1)';   % Initializing output signal according to equation (13)
OutputP = NoisyInput(1:AROrder,1);


i = AROrder+1;
j = AROrder+1;


for k = 1:NumSegments   % For every segment of chopped signal...
    jStart = j;     % Keeping track of AROrder+1 value for every iteration.
    OutputOld = OutputP;    % Keeping the first AROrder amount of samples for every iteration.
    
    for l = 1:NumIter
        A = [zeros(AROrder-1,1) eye(AROrder-1); fliplr(-FiltCoeff(k,2:end))];
        
        for ii = i:WinLenSamples
            OutputC = A * OutputP;
            Pc = (A * P * A') + (H' * Q(k) * H);
            K = (Pc * H')/((H * Pc * H') + R);
            OutputP = OutputC + (K * (ChoppedSignal(ii,k) - (H*OutputC)));
            Output(j-AROrder+1:j) = OutputP';
            P = (eye(AROrder) - K * H) * Pc;
            j = j+1;
        end
        
        i = 1;
        if l < NumIter
            j = jStart;
            OutputP = OutputOld;
        end
        
        % update lpc on filtered signal
        [FiltCoeff(k,:), Q(k)] = lpc(Output((k-1)*WinLenSamples+1:k*WinLenSamples),AROrder);
    end
end
Output = Output';

%disp(Output);

plot(handles.axes3,Time, Input)
xlabel(handles.axes3,'Time in seconds')
ylabel(handles.axes3,'Amlitude')
title(handles.axes3,'Clean speech signal')


plot(handles.axes2,Time, Noise)
xlabel(handles.axes2,'Time in seconds')
ylabel(handles.axes2,'Amlitude')
title(handles.axes2,'Observation noise')

plot(handles.axes4,Time, NoisyInput)
xlabel(handles.axes4,'Time in seconds')
ylabel(handles.axes4,'Amlitude')
title(handles.axes4,'Noisy input signal')

plot(handles.axes5,Time, Input)
xlabel(handles.axes5,'Time in seconds')
ylabel(handles.axes5,'Amlitude')
title(handles.axes5,'Estimated clean output signal')
%sound(Output,Fs);

% filename = 'handel.wav';
% audiowrite(filename,Output,Fs);


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Output Fs
% filename = 'threat_audio.wav';
% audiowrite(filename,Output,Fs);
f = msgbox('Audio file succesfully saved','Succes');
set(f, 'position', [420 120 180 70]);


% --- Executes on button press in pushbutton8.
function Play1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  o_player
im=imread('project_image.jpg');
axes(handles.axes6);
imshow(im)
[Input, Fs] = audioread('threat_audio.wav');
Input = Input(:,1);
o_player = audioplayer(Input, Fs);
play(o_player);
%sound(Output,Fs);


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global o_player
stop(o_player);

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global o_player
resume(o_player);

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global o_player
pause(o_player);

% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4
global Output Fs
contents = cellstr(get(hObject,'String'));
popupmemu3 = contents(get(hObject,'value'));
if (strcmp(popupmemu3,'0.5X'))
    player = audioplayer(Output, 0.75*Fs);
    play(player);
elseif (strcmp(popupmemu3,'1X'))
    player = audioplayer(Output, Fs);
    play(player);
elseif (strcmp(popupmemu3,'1.5X'))
    player = audioplayer(Output, 1.5*Fs);
    play(player);
elseif (strcmp(popupmemu3,'2X'))
    player = audioplayer(Output, 2*Fs);
    play(player);
end
assignin('base','player',player);

% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Input Fs
% [filename pathname] = uigetfile({'*.wav'}, 'Select File');
% fullfilename = strcat (pathname, filename);
[Input, Fs] = audioread('threat.wav');
%[Input,Fs] = audioread(filename);
im=imread('project_image.jpg');
axes(handles.axes1);
imshow(im);
f = msgbox('Audio file succesfully loaded','Succes');
set(f, 'position', [463 471 180 70]);
