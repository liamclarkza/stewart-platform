% Stewart Platform Test Code
% Author: Liam Clark
% 21 October, 2018

%% Run Test

stewie = StewartPlatform(152, 18.92, 103, 28.07, 73, 165);
stewie.connect('COM4');
stewie.calibrate;
% Perform x, y translation tests
for x = -20:10:20
    for y = -20:10:20
        stewie.set_position(0,0,0,x,y,140);
        stewie.move(0.75);
        !adb shell "input keyevent 27"
        pause(2);
    end
end
% Perform z translation tests
for z = 120:10:200
    stewie.set_position(0,0,0,0,0,z);
    stewie.move(0.75);
    take_photo;
end
% Perform roll tests
for roll = -30:10:30
    stewie.set_position(roll,0,0,0,0,140);
    stewie.move(0.75);
    take_photo;
end
% Perform pitch tests
for pitch = -30:10:30
    stewie.set_position(0,pitch,0,0,0,140);
    stewie.move(0.75);
    take_photo;
end
% Perform yaw tests
for yaw = -30:10:30
    stewie.set_position(0,0,yaw,0,0,140);
    stewie.move(0.75);
    take_photo;
end
            
%% Functions

% Rename images in 'Imports' folder
function rename()
    % X,Y Test Export
    outputFolder = fullfile(pwd, 'X, Y Test');
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end
    for x = -20:10:20
        for y = -20:10:20
            % Get the oldest file in Imports folder
            d = dir('Imports/*.jpg');
            if isempty(d)
                break;
            end
            filename = get_oldest_file(d);
            % Get full directory paths
            input_full_filename = fullfile(pwd,'Imports', filename);
            output_base_filename = sprintf('X%d Y%d.jpg', x, y);
            output_full_filename = fullfile(outputFolder, output_base_filename);
            copyfile(input_full_filename, output_full_filename);
            delete(input_full_filename);
        end
    end

    % Z Test Export
    outputFolder = fullfile(pwd, 'Z Test');
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end
    for z = 120:10:200
        % Get the oldest file in Imports folder
        d = dir('Imports/*.jpg');
        if isempty(d)
            break;
        end
        filename = get_oldest_file(d);
        % Get full directory paths
        input_full_filename = fullfile(pwd,'Imports', filename);
        output_base_filename = sprintf('Z%d.jpg', z);
        output_full_filename = fullfile(outputFolder, output_base_filename);
        copyfile(input_full_filename, output_full_filename);
        delete(input_full_filename);
    end
    % Roll Test Export
    outputFolder = fullfile(pwd, 'Roll Test');
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end
    for roll = 120:10:200
        % Get the oldest file in Imports folder
        d = dir('Imports/*.jpg');
        if isempty(d)
            break;
        end
        filename = get_oldest_file(d);
        % Get full directory paths
        input_full_filename = fullfile(pwd,'Imports', filename);
        output_base_filename = sprintf('Roll%d.jpg', roll);
        output_full_filename = fullfile(outputFolder, output_base_filename);
        copyfile(input_full_filename, output_full_filename);
        delete(input_full_filename);
    end
    % Pitch Test Export
    outputFolder = fullfile(pwd, 'Pitch Test');
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end
    for pitch = -30:10:30
        % Get the oldest file in Imports folder
        d = dir('Imports/*.jpg');
        if isempty(d)
            break;
        end
        filename = get_oldest_file(d);
        % Get full directory paths
        input_full_filename = fullfile(pwd,'Imports', filename);
        output_base_filename = sprintf('Pitch%d.jpg', pitch);
        output_full_filename = fullfile(outputFolder, output_base_filename);
        copyfile(input_full_filename, output_full_filename);
        delete(input_full_filename);
    end
    % Yaw Test Export
    outputFolder = fullfile(pwd, 'Yaw Test');
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end
    for yaw = -30:10:30
        % Get the oldest file in Imports folder
        d = dir('Imports/*.jpg');
        if isempty(d)
            break;
        end
        filename = get_oldest_file(d);
        % Get full directory paths
        input_full_filename = fullfile(pwd,'Imports', filename);
        output_base_filename = sprintf('Yaw%d.jpg', yaw);
        output_full_filename = fullfile(outputFolder, output_base_filename);
        copyfile(input_full_filename, output_full_filename);
        delete(input_full_filename);
    end
end

% Take photo using Android debugger
function take_photo()
    !adb shell "input keyevent 27"
    pause(2);
end

% Get the filename of oldest file in a directory
function filename = get_oldest_file(d)
    [~,idx] = min([d.datenum]);
    filename = d(idx).name;
end