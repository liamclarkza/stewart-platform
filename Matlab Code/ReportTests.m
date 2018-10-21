% Stewart Platform Routine Code
% Author: Liam Clark
% 21 October, 2018

%% Routine Code
% Perform initial cleanup
cleanup;
% Start Matlab mobile connector if not already running
if ~connector.isRunning
    connector on;
    pause(3);
end
create_timer;

function create_timer()
stewie = [];
m = [];
plot_enabled = [];
time = [];
log_vals = [];
log_time = [];
log_idx = [];
t=timer;
t.StartFcn = @initTimer;
t.TimerFcn = @timerCallback;
t.Period   = 0.025 ;
t.BusyMode = 'drop';
t.ExecutionMode  = 'fixedRate';
start(t);

    function initTimer(src, event)
        plot_enabled=0;
        % Initialise the platform and set up connections
        try
            stewie = StewartPlatform(152, 18.92, 103, 28.07, 62, 165);
            stewie.connect('COM4');
            stewie.calibrate;
        catch
            % If an error occurs, perform deconstruction
            cleanup;
        end
        % Begin plot
        time = 0;
        if(plot_enabled)
            stewie.plot_platform;
            drawnow;
        end
        % Move the platform into starting position to avoid an initial jolt
        [roll, pitch, yaw, x, y, z] = update(0);
        stewie.set_position(roll, pitch, yaw, x, y, z);
        stewie.move(2);
        pause(2);
        %preallocate memory for logging (assume 1 minute long tests)
        log_vals = zeros(60*1/t.Period, 6);
        log_time = zeros(60*1/t.Period, 1);
        log_idx = 1;
        %Set up mobile connection
        m = mobiledev;
        m.Logging = 0;
        m.discardlogs;
        
    end

    function timerCallback(src, event)
        if time == 0
            m.Logging = 1;
            tic
        end
        time = toc;
        [roll, pitch, yaw, x, y, z] = update(time);
        stewie.set_position(roll, pitch, yaw, x, y, z);
        log_time(log_idx,1) = toc;
        log_vals(log_idx,:) = [yaw, pitch, roll, x, y, z];
        stewie.move(t.Period);
        if(plot_enabled)
            stewie.update_plot;
            drawnow;
        end
        
        %check if exit key pressed
        if strcmp(get(gcf,'currentcharacter'),' ')
            toc
            stop(t);
            close(gcf);
            [orient, t_orient] = orientlog(m);
            
            time_in = log_time(1:log_idx, 1);
            input_vals = (log_vals(1:log_idx, 2:3 ));
            time_out = t_orient;
            output_vals = orient(:,2:3)-orient(1, 2:3);
            
            assignin('base','time_in',time_in);
            assignin('base','time_out',time_out);
            assignin('base','input_vals',input_vals);
            assignin('base','output_vals',output_vals);
            
            figure;
            %Plot input output
            subplot(2,1,1);
            hold on;
            plot(time_out, output_vals);
            plot(time_in, input_vals);
            legend('Output Pitch', 'Output Roll',...
                'Input Pitch', 'Input Roll');
            %xlim([0, 10]);
            xlim([0, 10]);
            ylim([-15 15]);
            title('Plot showing input and output pitch and roll vs time');
            xlabel('Time (sec)');
            ylabel('Degrees');
            set(gcf, 'Position', [500, 500, 600, 400]);
            hold off;
            %Plot error
            subplot(2,1,2);
            interp_input_vals = interp1(time_in, input_vals, time_out);
            plot(time_out, output_vals-interp_input_vals);
            xlim([0, 10]);
            ylim([-2 2]);
            title('Plot showing input and output pitch and roll error vs time');
            xlabel('Time (sec)');
            ylabel('Degrees');
            legend('Pitch Error', 'Roll Error');
            m.Logging = 0;
            m.discardlogs;
            stewie.set_position(0,0,0,0,0,115);
            stewie.move(2);
            
        end
        log_idx = log_idx+1;
    end

end


%% Functions

% Timer interrupt routine - Use this to define your routine!
function [roll, pitch, yaw, x, y, z] = update(time)
roll =  - 10*sind(360*(time^2)/10);
pitch = 10*sind(360*(time^2)/10);
yaw = 0;
x = 0;
y = 0;
z = 150;
end

% Delete old timers and unused serial connections
function cleanup()
% Find and delete old timers
for tim = timerfind
    tim.stop;
end
delete(timerfind);
% Delete all usused serial connections
if ~isempty(instrfind)
    fclose(instrfind);
end
delete(instrfind);
% Clear workspace variables
clear;
end