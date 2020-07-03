%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dallin Romney
% Ping Pong Launcher GUI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function varargout = Competition_Team_18(varargin)

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @Competition_Team_18_OpeningFcn, ...
                       'gui_OutputFcn',  @Competition_Team_18_OutputFcn, ...
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
    
% --- Outputs from this function are returned to the command line.
function varargout = Competition_Team_18_OutputFcn(hObject, eventdata, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAJOR FUNCTIONS:                                                                 %
% INITIALIZATION, COLOR PICKER, TOGGLE BUTTON, PRINT TO SERIAL, & COMPETITION_TEAM_18 CODE %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes just before Competition_Team_18 is made visible.
function Competition_Team_18_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to Competition_Team_18 (see VARARGIN)

    ClearAll(); %clear, clc

    % Choose default command line output for Competition_Team_18
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

    % Initialize listbox for serial monitor
    global listboxEntries
    listboxEntries = {'Waiting for commands'};

    %Clean up any existing serial port objects
    CleanUp(hObject, eventdata, handles);

    %Load the files containing data from last run.
    %I could have done this with one file but I think 3 works fine.
    load('SaveFiles/COM');
    load('SaveFiles/EncoderPos');
    load('SaveFiles/targetMapFile.mat');

    %Load the given image into the axes
    image(handles.targetMapAxes, targetMap);
    axis image

    %Update the edit boxes containing the saveable variables with the most
    %recent ones.
    handles.COMport.String = COMPort;
    handles.EncoderValue.String = adjustEncoderPos;
    handles.imageFileName.String = targetMapName;

    %Initialize the target table to all zeros
    updateCoordinates(hObject, eventdata, handles, zeros(1, 6), zeros(1, 6));
    updateTargets(hObject, eventdata, handles, zeros(1, 6), zeros(1, 6));      
        
function RGBvector = ColorPicker(hObject, eventdata, handles, array )
    %COLORPICKER pops up an image, takes the coordinates of a click, and
    %returns the RGB color values at the location of the click.
    %
    %Dallin Romney, u1087199, ME EN 1010, HW11

    global listboxEntries;     %Access the serial monitor

    %This try-catch structure just keeps trying until some valid colors are selected.
    while(1)
        try
            image(handles.targetMapAxes, array); %Display image
            axis image                           %Reproportion image

            %Optain the precise decimal coordinates of the user's click
            [x, y]  = ginput(1);                
            
            %Round the coordinates to the nearest integer and index out the pixel
            RGB = array(round(y), round(x), :);  
            
            %At this point, the code has worked. Break out of the loop.
            break;
        catch
            message = 'You must click on the image.';
            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);
        end
    end
    
    %Index out each value from the third dimension - otherwise RGB will be a 1x1x3 array.
    R = RGB(1, 1, 1);      
    G = RGB(1, 1, 2);
    B = RGB(1, 1, 3);
        
    %Concatenate the scalars to produce a 1x3 vector of RGB values
    RGBvector = [R, G, B]; 

function serial_toggle_Callback(hObject, eventdata, handles)

    %Disable toggle button. This prevents clicking it while the following
    %code is running. Button is enabled at completion
    set(hObject,'enable','off');
    
    %Access the serial monitor
    global listboxEntries;

    %Updates the COM port value from the text box and saves it to the file
    COMPort = str2double(handles.COMport.String);
    save('SaveFiles/COM', 'COMPort');

    if handles.serial_toggle.Value == 1
        try
            %Clear the serial port objects once again
            CleanUp(hObject, eventdata, handles);
            
            %Create the Romeo object and save it to a file for other functions to access.
            com = sprintf('COM%d', COMPort);          %The file only contains the number. We need a string.
            RomeoCOM = serial(com, 'BaudRate', 9600); %com is the port name, BaudRate is the property. 9600 is the value.
            save('SaveFiles/romeocom', 'RomeoCOM');

            %Open the object, and wait for the value 1 from it.
            fopen(RomeoCOM);
            fread(RomeoCOM,1);

            %Update the text on the button, change the color of the small rectangle, send a message to the monitor.
            handles.serial_toggle.String = 'Close port';
            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, ['Serial Port Status: Open']);
            handles.serialIndicator.BackgroundColor = [0, 1, 0];

            %Now that the Serial Port is up and running, enable the run button
            set(handles.RunCompetition,'enable','on');
            
        catch 
            %If the com port doesn't work, send a warning to the monitor and reset the button.
            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, 'The specified COM Port is not available');
            handles.serial_toggle.Value = 0;
        end
    else
        %Disable the main run button
        set(handles.RunCompetition,'enable','off');
        
        %Clean up any serial objects
        CleanUp(hObject, eventdata, handles);
        
        %Update the text on the button, the color of the small rectangle, and send a message to the monitor.
        handles.serial_toggle.String = 'Open port';
        listboxEntries = Print(hObject, eventdata, handles, listboxEntries, 'Serial Port Status: Closed');
        handles.serialIndicator.BackgroundColor = [1, 0, 0];
    end

    %Reenable toggle button
    set(hObject,'enable','on');
    
%This function prints a message to the list box / "Serial Monitor"
function listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message)
    currentEntry = length(listboxEntries) + 1;
    listboxEntries{currentEntry} = message;         %adds message to cell array
    handles.serial_listbox.String = listboxEntries; %display the cell array
    
    handles.serial_listbox.Value = currentEntry;
    drawnow
    
    fprintf('%s\n', message);
    
%THIS IS THE COMPETITION_TEAM_18 CODE
function RunCompetition_Callback(hObject, eventdata, handles)
    
    %Disables the Run button and enables the Stop button
    ChangeButtons(hObject, eventdata, handles, 'off');
    %Makes the listbox available
    global listboxEntries;
    
    %Reloads the target map string
    load('SaveFiles/targetMapFile');   
    
    %Updates the Adjust Encoder value from the edit box and saves it
    adjustEncoderPos = str2double(handles.EncoderValue.String);
    save('SaveFiles/EncoderPos', 'adjustEncoderPos');

    %This Try-Catch structure basically only allows colors that return 6 centroids
    while(1)
        try
            %Asks the user to click on the pokeball, extracts the colors at the
            %clicked point, and displays them.
            message = 'Click on the white half of the Pokeball';
            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);

            whiteRGB = ColorPicker(hObject, eventdata, handles, targetMap);
            message = sprintf('whiteRGB = %d %d %d', whiteRGB(1), whiteRGB(2), whiteRGB(3));
            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);

            %Same for colored half
            message = 'Click on the colored half of the Pokeball';
            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);

            coloredRGB = ColorPicker(hObject, eventdata, handles,targetMap);
            message = sprintf('coloredRGB = %d %d %d', coloredRGB(1), coloredRGB(2), coloredRGB(3));
            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);

            % Compute RGB values of the target rectangles
            targetRGB = whiteRGB - coloredRGB;
            message = sprintf('targetRGB = %d %d %d', targetRGB(1), targetRGB(2), targetRGB(3));
            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);

            %Find the target centroids and plot them on the picture, and the values
            %in the target table.

            [centroidRows, centroidCols, ~] = FindAllTargetCentroids( targetMap, targetRGB );
            updateCoordinates(hObject, eventdata, handles, centroidRows, centroidCols);
            break;
        catch
            message = 'You must click on the pokeball. Try again.';
            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);
        end
    end
    %Displays all the centroid values
    message = sprintf('centroidRows = %d %d %d %d %d %d', centroidRows(1), centroidRows(2), centroidRows(3), centroidRows(4), centroidRows(5), centroidRows(6));
    listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);
    message = sprintf('centroidCols = %d %d %d %d %d %d', centroidCols(1), centroidCols(2), centroidCols(3), centroidCols(4), centroidCols(5), centroidCols(6));
    listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);
    
    listboxEntries = Print(hObject, eventdata, handles, listboxEntries, 'The Romeo is calculating targets...');
    
    %Plots the centroid points on the picture
    hold on
    plot(handles.targetMapAxes,centroidCols, centroidRows, 'rX'); %Plot X's at all of the centroid coordinates
    pause(5)
    
	%%%%%%%%%%%%%%%%%%% TALK TO ROMEO %%%%%%%%%%%%%%%%%%%%%%%%
    %Loads and refreshes the Romeo Object
    %This is so you don't have to retoggle everytime you want to run.
    CleanUp(hObject, eventdata, handles);
    load('SaveFiles/romeocom');
    fclose(RomeoCOM);
    fopen(RomeoCOM);
    fread(RomeoCOM,1);
    
    % Compute values and write to Romeo
    xTarget_mm = zeros(1, 6);
    encoderPos = zeros(1, 6);
    
    %For each target:
    for k = 1:6
        %Convert the row value in mm to cm and adjust
        encoderPos(k) = centroidRows(k)/10 - adjustEncoderPos;
        
        %The target board starts 750 mm from the launcher
        xTarget_mm(k) = centroidCols(k) + 750;
        
        %Convert to base 256, because arduino can only recieve 0 to 255 with our code
        xTarget_HB = floor(xTarget_mm(k) / 256);
        xTarget_LB = xTarget_mm(k) - (256 * xTarget_HB);

        %Send the 3 values to the romeo board
        fwrite(RomeoCOM, encoderPos(k));
        fwrite(RomeoCOM, xTarget_HB);
        fwrite(RomeoCOM, xTarget_LB);
    end
    
    %Update the target table
    updateTargets(hObject, eventdata, handles, encoderPos, xTarget_mm/1000);

    % Scan for messages from romeo arduino loop
    while(RomeoCOM.BytesAvailable == 0)
    end
    
    message = fscanf(RomeoCOM); %Recieve the first message
    listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message(1:end-2));

    while(1)                             %Until broken
        if RomeoCOM.BytesAvailable > 0   %If there is a message available
            message = fscanf(RomeoCOM);  %Get it
            message = message(1:end-2);  %Cut off whitespace characters
            if isempty(message)          %End all receiving if it's an empty message
               break;
            end

            listboxEntries = Print(hObject, eventdata, handles, listboxEntries, message);
        end
    end

    %All done, so reenable the Buttons
    ChangeButtons(hObject, eventdata, handles, 'on');

    % disconnect SPO from Remeo and clear all serial objects

    
function loadImageButton_Callback(hObject, eventdata, handles)

    global listboxEntries; %Access the Serial Monitor
    
    try
        %Update the targetMap variable to hold whatever is in the corresponding edit box.
        targetMapName = handles.imageFileName.String;
        targetMap = imread(targetMapName);
        
        %Load the image into the axes
        image(targetMap);
        axis image;

        %Update the file containing the recently used string
        save('SaveFiles/targetMapFile', 'targetMap', 'targetMapName');
    catch
        %If there is an error, the file name is incorrect.
        listboxEntries = Print(hObject, eventdata, handles, listboxEntries, 'The image file name is invalid.');
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following code is Simple Button and Cleanup Functions %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Disables or enables all relevant buttons, depending on the value input: 'off' or 'on'
function ChangeButtons(hObject, eventdata, handles, value)
    set(handles.serial_toggle,    'enable', value);
    set(handles.incrementCOM,     'enable', value);
    set(handles.decrementCOM,     'enable', value);
    set(handles.incrementEncoder, 'enable', value);
    set(handles.decrementEncoder, 'enable', value);
    set(handles.loadImageButton,  'enable', value);
    set(handles.RunCompetition,   'enable', value);
 
%Checks if any serial objects exist and destroys them
function CleanUp(hObject, eventdata, handles)
    if length(instrfind) > 0
        fclose(instrfind)
        delete(instrfind)
    end

%Simply clears the workspace and the command window
%I was having problems with clear inside some other functions.
function ClearAll()
    clear
    clc
    
%This is the button that restarts the code
function quitButton_Callback(hObject, eventdata, handles)
    close
    ClearAll();
    Competition_Team_18

%This is the button that clears the Serial Monitor. It also clears the command window.
function clearSerialButton_Callback(hObject, eventdata, handles)
    clc

    %Access the serial monitor, and reinitialize it.
    global listboxEntries
    listboxEntries = {};
    listboxEntries = Print(hObject, eventdata, handles, listboxEntries, 'Waiting for commands');

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following code is for Increment/Decrement Buttons %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function incrementCOM_Callback(hObject, eventdata, handles)

    %Updates the saveable value of the most recently used COM port.
    comValue = str2double(handles.COMport.String);

    %Increments the value in the COM edit box. Limited to 12 ports.
    if comValue < 12
        handles.COMport.String = num2str(comValue + 1);
    else
        handles.COMport.String = num2str(12);
    end

function decrementCOM_Callback(hObject, eventdata, handles)

    %Updates the saveable value of the most recently used COM port.
    comValue = str2double(handles.COMport.String);

    %Increments the value in the COM edit box. You can't have a negative port.
    if comValue > 0
        handles.COMport.String = num2str(comValue - 1);
    else
        handles.COMport.String = num2str(0);
    end

function incrementEncoder_Callback(hObject, eventdata, handles)

    %Updates the saveable value of the most recently used adjust encoder
    encoderValue = str2double(handles.EncoderValue.String);

    %Increments the value in the Encoder Pos edit box. Limits to 10 cm
    if encoderValue < 10
        handles.EncoderValue.String = num2str(encoderValue + 0.1);
    else
        handles.EncoderValue.String = num2str(10);
    end

function decrementEncoder_Callback(hObject, eventdata, handles)

    %Updates the saveable value of the most recently used adjust encoder
    encoderValue = str2double(handles.EncoderValue.String);

    %Increments the value in the Encoder Pos edit box. Limits to 0 cm
    if encoderValue > 0
        handles.EncoderValue.String = num2str(encoderValue - 0.1);
    else
        handles.EncoderValue.String = num2str(0);
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following code is for the target Table %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This function simply takes two vectors and displays them into the X and Y
%columns of the Target Table. I could have used a table, but we didn't
%learn those and I was having a pretty fundamental problem when I tried.
function updateCoordinates(hObject, eventdata, handles, vector1, vector2)

    handles.target1X.String = num2str(vector1(1));
    handles.target2X.String = num2str(vector1(2));
    handles.target3X.String = num2str(vector1(3));
    handles.target4X.String = num2str(vector1(4));
    handles.target5X.String = num2str(vector1(5));
    handles.target6X.String = num2str(vector1(6));
    
    handles.target1Y.String = num2str(vector2(1));
    handles.target2Y.String = num2str(vector2(2));
    handles.target3Y.String = num2str(vector2(3));
    handles.target4Y.String = num2str(vector2(4));
    handles.target5Y.String = num2str(vector2(5));
    handles.target6Y.String = num2str(vector2(6));

%This does the same as updateCoordinates, except it updates the "stipe" and
%"dist" columns.
function updateTargets(hObject, eventdata, handles, vector1, vector2)

    handles.target1Stripe.String = num2str(vector1(1));
    handles.target2Stripe.String = num2str(vector1(2));
    handles.target3Stripe.String = num2str(vector1(3));
    handles.target4Stripe.String = num2str(vector1(4));
    handles.target5Stripe.String = num2str(vector1(5));
    handles.target6Stripe.String = num2str(vector1(6));
    
    handles.target1Dist.String = num2str(vector2(1));
    handles.target2Dist.String = num2str(vector2(2));
    handles.target3Dist.String = num2str(vector2(3));
    handles.target4Dist.String = num2str(vector2(4));
    handles.target5Dist.String = num2str(vector2(5));
    handles.target6Dist.String = num2str(vector2(6));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following code only effects colors on startup. %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function target1X_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target1Y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target2X_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target2Y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target3X_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target3Y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target4X_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target4Y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target5X_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target5Y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target6X_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target6Y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit17_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit18_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit19_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target1Stripe_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target1Dist_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target2Stripe_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target2Dist_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target3Stripe_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target3Dist_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target4Stripe_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target4Dist_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target5Stripe_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target5Dist_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target6Stripe_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function target6Dist_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function sStripeHeader_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit33_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function serial_listbox_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function EncoderValue_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function COMport_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function imageFileName_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following are callbacks designed to do absolutely nothing.%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function imageFileName_Callback(hObject, eventdata, handles)

function COMport_Callback(hObject, eventdata, handles)

function EncoderValue_Callback(hObject, eventdata, handles)
