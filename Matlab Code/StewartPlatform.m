% Stewart Platform Class
% Author: Liam Clark
% 21 October, 2018

%% Stewart Platform Class
classdef StewartPlatform < handle
    properties
        %Platform coordinates and rotations
        roll
        pitch
        yaw
        x
        y
        z
        
        %Platform configuration properties
        base_radius
        base_angle
        platform_radius
        platform_angle
        lower_leg_length
        upper_leg_length
        stepper_plane_angle
        
        %Motor angles
        ang
        
        %Coordinates of points
        P %points where legs attach to the platform relative to the platform
        Q %points where legs attach to the platform relative to the base
        B %points where legs attach to the base relative to the base
        A %points where leg linkages attach to one another relative to the base
        
        %Figure handle
        fig
        
        %Serial connection
        ser
    end
    
    methods
        %% Constructor Function
        function obj = StewartPlatform(base_radius, base_angle, platform_radius, platform_angle, lower_leg_length, upper_leg_length)
            %Initialise to zero position
            obj.roll = 0;
            obj.pitch = 0;
            obj.yaw = 0;
            obj.x = 0;
            obj.y = 0;
            obj.z = 105;
            %Initialise motors
            obj.ang = zeros(1,6); %these are the angles of the stepper motors relative to the x-y plane
            %Set object configuration properties
            obj.base_radius = base_radius;
            obj.base_angle = base_angle;
            obj.platform_radius = platform_radius;
            obj.platform_angle = platform_angle;
            obj.lower_leg_length = lower_leg_length;
            obj.upper_leg_length = upper_leg_length;
            obj.stepper_plane_angle = [-90    90   120-90     120+90     -120-90    -120+90];
            %Preallocate memory for graphics objects
            obj.fig = gobjects;
            %Define the base-leg joint coordinates
            obj.B = obj.base_radius.*[cosd(-obj.base_angle/2)   cosd(obj.base_angle/2)  cosd(-obj.base_angle/2 + 120)   cosd(obj.base_angle/2 +120)     cosd(-obj.base_angle/2 - 120)   cosd(obj.base_angle/2 - 120);
                sind(-obj.base_angle/2)   sind(obj.base_angle/2)  sind(-obj.base_angle/2 + 120)   sind(obj.base_angle/2 +120)     sind(-obj.base_angle/2 - 120)   sind(obj.base_angle/2 - 120);
                0                         0                       0                               0                               0                               0];
            %Define the platform-leg joint coordinates
            obj.P = obj.platform_radius.*[cosd(-obj.platform_angle/2)   cosd(obj.platform_angle/2)  cosd(-obj.platform_angle/2 + 120)   cosd(obj.platform_angle/2 +120)     cosd(-obj.platform_angle/2 - 120)   cosd(obj.platform_angle/2 - 120);
                sind(-obj.platform_angle/2)   sind(obj.platform_angle/2)  sind(-obj.platform_angle/2 + 120)   sind(obj.platform_angle/2 +120)     sind(-obj.platform_angle/2 - 120)   sind(obj.platform_angle/2 - 120);
                0                             0                           0                                   0                                   0                                   0];
            %Determine coordinates
            set_position(obj, obj.roll, obj.pitch, obj.yaw, obj.x, obj.y, obj.z);
        end
        
        %% Rotation Matrix Calculation Function
        function R = get_rotation_matrix(obj)
            %Rotation about Z-axis (Yaw)
            Rz = [cosd(obj.yaw)     -sind(obj.yaw)  0;
                sind(obj.yaw)     cosd(obj.yaw)   0;
                0             0           1];
            %Rotation about-axis Y (Pitch)
            Ry = [cosd(obj.pitch)   0           sind(obj.pitch);
                0             1           0;
                -sind(obj.pitch)  0           cosd(obj.pitch)];
            %Rotation about-axis X (Roll)
            Rx = [1             0           0;
                0             cosd(obj.roll)  -sind(obj.roll);
                0             sind(obj.roll)  cosd(obj.roll)];
            
            R = Rz*Ry*Rx;
        end
        
        %% Draw the platform in its current configuration
        function fig = plot_platform(obj)
            clf;
            hold on;
            fill3([obj.Q(1,:) obj.Q(1,1)],[obj.Q(2,:) obj.Q(2,1)],[obj.Q(3,:) obj.Q(3,1)],'red');
            fill3([obj.B(1,:) obj.B(1,1)],[obj.B(2,:) obj.B(2,1)],[obj.B(3,:) obj.B(3,1)],'red');
            for i = 1:6
                plot3([obj.B(1,i), obj.Q(1,i)], [obj.B(2,i), obj.Q(2,i)], [obj.B(3,i), obj.Q(3,i)], 'color', [0.7 0.7 0.7],'linestyle', '--', 'linewidth', 1);
            end
            for i = 1:6
                plot3([obj.B(1,i), obj.A(1,i)], [obj.B(2,i), obj.A(2,i)], [obj.B(3,i), obj.A(3,i)], 'color', 'black', 'linewidth', 3);
                plot3([obj.A(1,i), obj.Q(1,i)], [obj.A(2,i), obj.Q(2,i)], [obj.A(3,i), obj.Q(3,i)], 'color', 'black', 'linewidth', 3);
            end
            xlim([-obj.base_radius-50 obj.base_radius+50]);
            ylim([-obj.base_radius-50 obj.base_radius+50]);
            zlim([-50 2*(obj.base_radius)]);
            view(135,30);
            view(255,30);
            axis vis3d;
            grid on;
            obj.fig = gcf;
            fig = obj.fig;
            hold off;
        end
        
        %% Update the plot
        function fig = update_plot(obj)
            cla(obj.fig);
            hold on;
            fill3([obj.Q(1,:) obj.Q(1,1)],[obj.Q(2,:) obj.Q(2,1)],[obj.Q(3,:) obj.Q(3,1)],'red');
            fill3([obj.B(1,:) obj.B(1,1)],[obj.B(2,:) obj.B(2,1)],[obj.B(3,:) obj.B(3,1)],'red');
            plot3([obj.B(1,:); obj.A(1,:)], [obj.B(2,:); obj.A(2,:)], [obj.B(3,:); obj.A(3,:)], 'color', 'black', 'linewidth', 3);
            plot3([obj.A(1,:); obj.Q(1,:)], [obj.A(2,:); obj.Q(2,:)], [obj.A(3,:); obj.Q(3,:)], 'color', 'black', 'linewidth', 3);
            obj.fig = gcf;
            fig = obj.fig;
            hold off;
        end
        
        %% Set the position and angular variables of the platform
        function valid = set_position(obj, roll, pitch, yaw, x, y, z)
            
            obj.roll = roll;
            obj.pitch = pitch;
            obj.yaw = yaw;
            obj.x = x;
            obj.y = y;
            obj.z = z;
            
            R = get_rotation_matrix(obj);
            valid = 1;
            
            for i = 1:6
                obj.Q(:,i) = R*obj.P(:,i) + [obj.x; obj.y; obj.z];
                L = sum((obj.Q(:,i) - obj.B(:,i)).^2) - (obj.upper_leg_length^2 - obj.lower_leg_length^2);
                M = 2*obj.lower_leg_length*(obj.Q(3,i) - obj.B(3,i));
                N = 2*obj.lower_leg_length*((cosd(obj.stepper_plane_angle(i))*(obj.Q(1,i) - obj.B(1,i))) + (sind(obj.stepper_plane_angle(i))*(obj.Q(2,i) - obj.B(2,i))));
                obj.ang(i) = asind(L/sqrt(M^2 + N^2)) - atand(N/M);
                if ~isreal(obj.ang(i))
                    %disp("Invalid platform configuration")
                    valid = 0;
                end
            end
            %disp(ang);
            
            for i = 1:6
                obj.A(:,i) = [obj.lower_leg_length*cosd(obj.ang(i))*cosd(obj.stepper_plane_angle(i)) + obj.B(1,i);
                    obj.lower_leg_length*cosd(obj.ang(i))*sind(obj.stepper_plane_angle(i)) + obj.B(2,i);
                    obj.lower_leg_length*sind(obj.ang(i)) + obj.B(3,i);];
            end
        end
        
        %% Connect to platform via serial
        function connect(obj, serial_port_string)
            if ~isempty(instrfind)
                fclose(instrfind);
            end
            delete(instrfind);
            %to find serial ports connected - run 'seriallist'
            if ismac
                obj.ser = serial(serial_port_string);
            elseif isunix
                obj.ser = serial(serial_port_string);
            elseif ispc
                obj.ser = serial(serial_port_string);
            end
            fopen(obj.ser);
        end
        
        %% Disconnect from platform
        function disconnect(obj)
            fclose(obj.ser);
            delete(obj.ser);
        end
        %% Send move command to platform
        function move(obj, time)
            a = zeros(1,6);
            for i = 1:6
                a(i) = round(obj.ang(i) * 6400/360);
            end
            msg = sprintf('m %f %d %d %d %d %d %d;', time, a(1), -a(2), a(3), -a(4), a(5), -a(6));
            fwrite(obj.ser, msg);
        end
        
        %% Direct actuator write command to platform
        function set_actuator(obj, time, m1, m2, m3, m4, m5, m6)
            msg = sprintf('m %f %d %d %d %d %d %d;', time, m1, m2, m3, m4, m5, m6);
            fwrite(obj.ser, msg);
        end
        
        %% Send reset command to platform
        function reset(obj)
            fwrite(obj.ser, 'r;');
        end
        
        %% Calibrate
        function calibrate(obj)
            reset(obj);
            cal = 590;
            set_actuator(obj, 1, cal, -cal, cal, -cal, cal, -cal);
            pause(1.2);
            reset(obj);
        end
    end
end