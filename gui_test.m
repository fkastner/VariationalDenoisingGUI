function varargout = gui_test(varargin)
% GUI_TEST MATLAB code for gui_test.fig
%      GUI_TEST, by itself, creates a new GUI_TEST or raises the existing
%      singleton*.
%
%      H = GUI_TEST returns the handle to a new GUI_TEST or the handle to
%      the existing singleton*.
%
%      GUI_TEST('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_TEST.M with the given input arguments.
%
%      GUI_TEST('Property','Value',...) creates a new GUI_TEST or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_test_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_test_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_test

% Last Modified by GUIDE v2.5 18-Jun-2017 11:17:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_test_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_test_OutputFcn, ...
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

%#ok<*DEFNU> %suppress function not called
%#ok<*INUSL> %suppress parameter not used
%#ok<*INUSD>

% --- Executes just before gui_test is made visible.
function gui_test_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_test (see VARARGIN)

% Set Colormap to gray for intensity images
colormap(handles.axes_noise,gray(255));
colormap(handles.axes_denoised,gray(255));

% Load the test data
load('../data/testimages.mat');
handles.img_orig = double(cameraman100);
handles.img_noise = handles.img_orig;
handles.img_denoised = handles.img_orig;

% Initialize rectangle selection for histogram
fcn = makeConstrainToRectFcn('impoly',handles.axes_noise.XLim,handles.axes_noise.YLim);
handles.hist_poly = impoly(handles.axes_noise, [1,1; 20,1; 20,20; 1,20], 'PositionConstraintFcn',fcn);

% Initialize current noise model
noise_model = struct();
noise_model.fixedrngseed = true; % Use fixed seed for reproducible noise
noise_model.dist = 'Normal'; % Initial distribution
noise_model.param1 = handles.slider_param1.Value;
noise_model.param2 = handles.slider_param2.Value;
handles.noise_model = noise_model;

handles.text_param1.String = ['Param1: ', num2str(round(handles.slider_param1.Value))];
handles.text_param2.String = ['Param2: ', num2str(round(handles.slider_param2.Value))];
updateNoise(hObject, handles);
handles = guidata(hObject);

% Initialize denoising
params = struct();
params.debugoutput = true; % Enable debug output
params.fidelity = 'L1'; % Initial fidelity
params.regularizer = 'TV'; % Initial regularizer
params.discrep = true;
params.alpha = handles.slider_denoise_alpha.Value;
params.conflevel = handles.slider_conflevel.Value;
params.sample_quantiles = false;
params.debugoutput = true;
handles.denoise_params = params;

handles.text_alpha.String = ['Alpha: ' num2str(round(handles.slider_denoise_alpha.Value,3))];

% Choose default command line output for gui_test
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui_test wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_test_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in checkbox_noise.
function checkbox_noise_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_noise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_noise

if handles.checkbox_noise.Value
    image(handles.axes_noise,uint8(handles.img_noise));
    fcn = makeConstrainToRectFcn('impoly',handles.axes_noise.XLim,handles.axes_noise.YLim);
    handles.hist_poly = impoly(handles.axes_noise, handles.poly_pos, 'PositionConstraintFcn',fcn);
    title(handles.axes_noise, {['PSNR: ' num2str(handles.psnr_noisy)]; [' SSIM: ' num2str(handles.ssim_noisy)]});
else
    handles.poly_mask = createMask(handles.hist_poly);
    handles.poly_pos = getPosition(handles.hist_poly);
    image(handles.axes_noise,uint8(handles.img_orig));
    title(handles.axes_noise, 'Original');
end

guidata(hObject, handles);


% --- Executes on slider movement.
function slider_param1_Callback(hObject, eventdata, handles)
% hObject    handle to slider_param1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

handles.text_param1.String = ['Param1: ', num2str(round(handles.slider_param1.Value,2))];
handles.noise_model.param1 = round(handles.slider_param1.Value,2);
updateNoise(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider_param1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_param1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on selection change in popupmenu_dist.
function popupmenu_dist_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_dist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_dist contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_dist
contents = cellstr(handles.popupmenu_dist.String);
handles.noise_model.dist = contents{handles.popupmenu_dist.Value};

switch handles.noise_model.dist
    case {'Normal','Uniform','Rician'}
        handles.slider_param1.Min = 0.0;
        handles.slider_param1.Max = 255.0;
        handles.slider_param2.Enable = 'on';
    case {'Exponential','Poisson'}
        handles.slider_param1.Min = 0.0;
        handles.slider_param1.Max = 255.0;
        handles.slider_param2.Enable = 'off';
    case {'Salt & Pepper'}
        handles.slider_param1.Value = 0.5;
        handles.noise_model.param1 = handles.slider_param1.Value;
        handles.slider_param1.Min = 0.0;
        handles.slider_param1.Max = 1.0;
        handles.slider_param2.Enable = 'off';        
end

% handles.current_dist = contents{handles.popupmenu_dist.Value};

updateNoise(hObject, handles);

% --- Executes during object creation, after setting all properties.
function popupmenu_dist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_dist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- To be called when noise related settings change.
function updateNoise(hObject, handles)
% Update the noise
handles.img_noise = addNoise(handles.img_orig,handles.noise_model);
handles.noise = handles.img_noise - handles.img_orig;

% Update image display
if handles.checkbox_noise.Value
    % Draw rectangle for background region at the same position it was before
    pos = getPosition(handles.hist_poly);
    image(handles.axes_noise,uint8(handles.img_noise));
    fcn = makeConstrainToRectFcn('impoly',handles.axes_noise.XLim,handles.axes_noise.YLim);
    handles.hist_poly = impoly(handles.axes_noise, pos, 'PositionConstraintFcn',fcn);
    % Update PSNR in title of axes
    handles.psnr_noisy = psnr(handles.img_noise, handles.img_orig, 255);
    handles.ssim_noisy = ssim(handles.img_noise, handles.img_orig, 'DynamicRange',255);
    title(handles.axes_noise, {['PSNR: ' num2str(handles.psnr_noisy)]; [' SSIM: ' num2str(handles.ssim_noisy)]});
else
    image(handles.axes_noise,uint8(handles.img_orig));
    title(handles.axes_noise, 'Original');
end

% Update histogram display
btn_hist_Callback(hObject, [], handles);

% --- Executes on slider movement.
function slider_param2_Callback(hObject, eventdata, handles)
% hObject    handle to slider_param2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.text_param2.String = ['Param2: ', num2str(round(handles.slider_param2.Value))];
handles.noise_model.param2 = round(handles.slider_param2.Value);
updateNoise(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider_param2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_param2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function axes_hist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes_hist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes_hist


% --- Executes on button press in btn_hist.
function btn_hist_Callback(hObject, eventdata, handles)
% hObject    handle to btn_hist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% update histogram display
% if isvalid(handles.hist_poly)
%     pos = getPosition(handles.hist_poly);
% else
%     pos = handles.poly_pos;
% end
% handles.noise_model.bg_noise = getRegionNoise(handles.noise, handles.hist_poly, ~handles.checkbox_outsidevalues.Value);
if isvalid(handles.hist_poly)
    Idx = createMask(handles.hist_poly);
else
    Idx = handles.poly_mask;
end
if handles.checkbox_outsidevalues.Value
    Idx = ~Idx;
end
handles.noise_model.bg_noise = handles.noise(Idx);
handles.hist = histogram(handles.axes_hist,handles.noise_model.bg_noise,'Normalization','pdf');
handles.denoise_params.std = std(handles.noise_model.bg_noise);

% update pdf plot
lim = handles.hist.BinLimits;
x = linspace(lim(1),lim(2),100);

switch handles.noise_model.dist
    case {'Normal','Uniform','Rician'}
        y = pdf(handles.noise_model.dist,x,handles.noise_model.param1,...
            handles.noise_model.param2);
    case 'Exponential'
        y = pdf(handles.noise_model.dist,x,handles.noise_model.param1);
    case 'Poisson'
        x = round(x);
        y = pdf(handles.noise_model.dist,x,handles.noise_model.param1);
    otherwise
        y = zeros(size(x));
end
hold(handles.axes_hist,'on');
plot(handles.axes_hist,x,y);
hold(handles.axes_hist,'off');

guidata(hObject, handles);


% --- Executes on button press in checkbox_outsidevalues.
function checkbox_outsidevalues_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_outsidevalues (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_outsidevalues


% --- Executes on button press in btn_denoise.
function btn_denoise_Callback(hObject, eventdata, handles)
% hObject    handle to btn_denoise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Update background noise first
btn_hist_Callback(hObject, [], handles);
handles = guidata(hObject);

% Special parameters for different fidelity functions
% switch handles.denoise_params.fidelity
%     case {'Bounds', 'Relaxed Bounds L1', 'Relaxed Bounds L2sq'}
%         [lower,upper] = getBounds(handles.img_noise, handles.noise_model, handles.conflevel);
%         handles.denoise_params.lowerbounds = lower;
%         handles.denoise_params.upperbounds = upper;
%         handles.denoise_params.delta = -1;
%     case {'L2', 'Residual'}
%         handles.denoise_params.delta = handles.noise_model.param2 * sqrt(length(handles.img_noise(:)));
%     case {'L1'}
%         handles.denoise_params.delta = handles.noise_model.param2 * length(handles.img_noise(:));
%     case 'L2 squared'
%         handles.denoise_params.delta = handles.noise_model.param2^2 * length(handles.img_noise(:));
% end

% Do the denoising
handles.text_status.String = 'Working ...'; drawnow;
tic;
[handles.img_denoised, stats] = denoise(handles.img_noise,handles.denoise_params,handles.noise_model);
toc
fprintf('Alpha: %f\n',stats.alpha);
handles.text_status.String = stats.cvx_status;

% Update alpha slider and text
handles.slider_denoise_alpha.Value = round(stats.alpha,3);
handles.text_alpha.String = ['Alpha: ', num2str(round(stats.alpha,3))];

% Update image display
image(handles.axes_denoised, handles.img_denoised);
handles.psnr_denoised = psnr(handles.img_denoised, handles.img_orig, 255);
handles.ssim_denoised = ssim(handles.img_denoised, handles.img_orig, 'DynamicRange',255);
title(handles.axes_denoised, {['PSNR: ' num2str(handles.psnr_denoised)]; [' SSIM: ' num2str(handles.ssim_denoised)]});

guidata(hObject,handles);

% --- Executes on slider movement.
function slider_denoise_alpha_Callback(hObject, eventdata, handles)
% hObject    handle to slider_denoise_alpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.text_alpha.String = ['Alpha: ', num2str(round(handles.slider_denoise_alpha.Value,3))];
handles.denoise_params.alpha = round(handles.slider_denoise_alpha.Value,3);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function slider_denoise_alpha_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_denoise_alpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on selection change in popupmenu_regularizer.
function popupmenu_regularizer_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_regularizer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_regularizer contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_regularizer
contents = cellstr(handles.popupmenu_regularizer.String);
handles.denoise_params.regularizer = contents{handles.popupmenu_regularizer.Value};
switch handles.denoise_params.regularizer
    case 'TVLp-hom'
        handles.slider_p.Enable = 'on';
        handles.slider_beta.Enable = 'on';
    case {'Residual', 'Bounds', 'Relaxed Bounds L1', 'Relaxed Bounds L2sq'}
        handles.slider_p.Enable = 'off';
        handles.slider_beta.Enable = 'off';
    otherwise
        handles.slider_p.Enable = 'off';
        handles.slider_beta.Enable = 'off';
end

guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function popupmenu_regularizer_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_regularizer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in popupmenu_fidelity.
function popupmenu_fidelity_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_fidelity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_fidelity contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_fidelity
contents = cellstr(handles.popupmenu_fidelity.String);
handles.denoise_params.fidelity = contents{handles.popupmenu_fidelity.Value};

switch handles.denoise_params.fidelity
    case {'Bounds', 'Relaxed Bounds Residual'}
        handles.slider_conflevel.Enable = 'on';
        handles.checkbox_sample_quantiles.Enable = 'on';
        handles.slider_denoise_alpha.Enable = 'off';
        handles.checkbox_discrep.Value = 0.0;
        handles.checkbox_discrep.Enable = 'off';
        handles.denoise_params.discrep = false;
    case {'Relaxed Bounds L1', 'Relaxed Bounds L2sq'}
        handles.slider_conflevel.Enable = 'on';
        handles.checkbox_sample_quantiles.Enable = 'on';
        handles.slider_denoise_alpha.Enable = 'on';
        handles.checkbox_discrep.Value = 0.0;
        handles.checkbox_discrep.Enable = 'off';
        handles.denoise_params.discrep = false;
    case {'Residual'}
        handles.slider_conflevel.Enable = 'off';
        handles.checkbox_sample_quantiles.Enable = 'off';
        handles.slider_denoise_alpha.Enable = 'off';
        handles.checkbox_discrep.Value = 0.0;
        handles.checkbox_discrep.Enable = 'off';
        handles.denoise_params.discrep = false;
    otherwise
        handles.slider_conflevel.Enable = 'off';
        handles.checkbox_sample_quantiles.Enable = 'off';
        handles.slider_denoise_alpha.Enable = 'off';
        handles.checkbox_discrep.Value = 1.0;
        handles.checkbox_discrep.Enable = 'on';
        handles.denoise_params.discrep = true;
end

guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function popupmenu_fidelity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_fidelity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_discrep.
function checkbox_discrep_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_discrep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_discrep
if handles.checkbox_discrep.Value
    handles.denoise_params.discrep = true;
    handles.slider_denoise_alpha.Enable = 'off';
else
    handles.denoise_params.discrep = false;
    if handles.slider_denoise_alpha.Value > handles.slider_denoise_alpha.Max || handles.slider_denoise_alpha.Value < handles.slider_denoise_alpha.Min
        handles.slider_denoise_alpha.Value = 0.5*handles.slider_denoise_alpha.Max + 0.5*handles.slider_denoise_alpha.Min;
        handles.text_alpha.String = ['Alpha: ', num2str(round(handles.slider_denoise_alpha.Value,3))];
        handles.denoise_params.alpha = round(handles.slider_denoise_alpha.Value,3);
    end
    
    handles.slider_denoise_alpha.Enable = 'on';
end

guidata(hObject, handles);


% --- Executes on slider movement.
function slider_conflevel_Callback(hObject, eventdata, handles)
% hObject    handle to slider_conflevel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.text_conflevel.String = ['Conf Lvl: ', num2str(round(handles.slider_conflevel.Value,3))];
handles.conflevel = round(handles.slider_conflevel.Value,3);
handles.denoise_params.conflevel = round(handles.slider_conflevel.Value,3);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_conflevel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_conflevel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_beta_Callback(hObject, eventdata, handles)
% hObject    handle to slider_beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.text_beta.String = ['Beta: ', num2str(round(handles.slider_beta.Value,3))];
handles.denoise_params.beta = round(handles.slider_beta.Value,3);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_beta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_p_Callback(hObject, eventdata, handles)
% hObject    handle to slider_p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.text_p.String = ['p: ', sprintf('%.0f',handles.slider_p.Value)];
handles.denoise_params.p = round(handles.slider_p.Value);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_p_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in checkbox_fixedrngseed.
function checkbox_fixedrngseed_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_fixedrngseed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_fixedrngseed
handles.noise_model.fixedrngseed = logical(handles.checkbox_fixedrngseed.Value);
guidata(hObject,handles);



function edit_beta_Callback(hObject, eventdata, handles)
% hObject    handle to edit_beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_beta as text
%        str2double(get(hObject,'String')) returns contents of edit_beta as a double

% handles.text_beta.String = ['Beta: ', num2str(round(handles.slider_beta.Value,3))];
handles.denoise_params.beta = str2double(handles.edit_beta.String);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_beta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_testimage.
function popupmenu_testimage_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_testimage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

contents = cellstr(handles.popupmenu_testimage.String);
name = contents{handles.popupmenu_testimage.Value};
if ~strcmp(name, 'Custom')
    load('../data/testimages.mat');
    handles.img_orig = double(eval(name));
else
    sample_image_folder = fullfile(matlabroot,'toolbox/images/imdata');
    [filename,user_canceled] = imgetfile('InitialPath',sample_image_folder);
    if ~user_canceled
        [temp,map] = imread(filename);
        if isempty(map) && ndims(temp) == 3
            temp = rgb2gray(temp);
        elseif ndims(temp) == 3
            temp = ind2gray(temp, map);
        end
        temp = double(temp);
%         if numel(temp) > 10100
%             temp = imresize(temp, 100/sqrt(numel(temp))); % scale to approx. 10000 pixels
%         end
        handles.img_orig = temp;
    end
end

updateNoise(hObject, handles);


% --- Executes during object creation, after setting all properties.
function popupmenu_testimage_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_testimage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_sample_quantiles.
function checkbox_sample_quantiles_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_sample_quantiles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_sample_quantiles
handles.denoise_params.sample_quantiles = logical(handles.checkbox_sample_quantiles.Value);
guidata(hObject,handles);
