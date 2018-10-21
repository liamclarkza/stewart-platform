% Graph Generating Code
% Author: Liam Clark
% 21 October, 2018

%% Interpolated stepper positions
x = 0:30:360;
y = round(10*sind(x));
xi = 0:0.1:360;
yi = round(interp1(x,y,xi));
figure;
hold on
scatter(x/36,y);
plot(x/36,y);
plot(xi/36,yi);
hold off
xlabel('Time');
ylabel('Motor Position (steps)');
legend('Points where microcontroller sent new move command', 'Interpolated ideal position', 'Stepper motor position');
title('Plot showing position of stepper motor tracking a sin wave')
xlim([0 10]);
ylim([-15 15]);
hold off;

%% Stepper movement
x = 0:10;
y = (0:10)*7/10;
y_step = round(y);
figure;
hold on;
plot(x,y);
stairs(x,y_step);
hold off;
xlabel('Time');
ylabel('Motor Position (steps)');
legend('Ideal Position', 'Stepper Motor Position');
xlim([0 10]);
ylim([0 8.5]);
title('Plot showing minimum error path for discretisation of position');
hold off;

%% Pitch Tests
load('Tests');
figure;
hold on;
plot([-20 20], [-20 20],'--');
scatter(pitchunloaded.inputpitchunloaded, pitchunloaded.outputpitchunloaded,'o');
scatter(pitchloaded.inputpitchloaded, pitchloaded.outputpitchloaded,'*');
title('Plot showing input vs output for pitch test');
xlabel('Input Pitch (degrees)');
ylabel('Output Pitch (degrees)');
legend('Ideal response', 'Response (no load)', 'Response (with load)');
hold off;

%% Roll Tests
load('Tests');
figure;
hold on;
plot([-30 30], [-30 30],'--');
scatter(rollunloaded.inputrollunloaded, rollunloaded.outputrollunloaded,'o');
scatter(rollloaded.inputrollloaded, rollloaded.outputrollloaded,'*');
title('Plot showing input vs output for roll test');
xlabel('Input Roll (degrees)');
ylabel('Output Roll (degrees)');
legend('Ideal response', 'Response (no load)', 'Response (with load)');
hold off;

%% Yaw Tests
load('Tests');
figure;
hold on;
plot([-30 30], [-30 30],'--');
scatter(yawunloaded.inputyawunloaded, yawunloaded.outputyawunloaded,'o');
scatter(yawloaded.inputyawloaded, yawloaded.outputyawloaded,'*');
title('Plot showing input vs output for yaw test');
xlabel('Input Yaw (degrees)');
ylabel('Output Yaw (degrees)');
legend('Ideal response', 'Response (no load)', 'Response (with load)');
hold off;

%% XY Tests
load('Tests');
figure;
hold on;
scatter(xyunloaded.inputxunloaded, xyunloaded.inputyunloaded,'o');
scatter(xyunloaded.outputxunloaded, xyunloaded.outputyunloaded,'*');
scatter(xyloaded.outputxloaded, xyloaded.outputyloaded,'+');
title('Plot showing input vs output for XY test');
xlabel('X (mm)');
ylabel('Y (mm)');
legend('Ideal response', 'Response (no load)', 'Response (with load)');
hold off;

%% Z Tests
load('Tests');
figure;
hold on;
plot([120 200], [120 200],'--');
scatter(zunloaded.inputzunloaded, zunloaded.outputzunloaded,'o');
scatter(zloaded.inputzloaded, zloaded.outputzloaded,'*');
title('Plot showing input vs output for Z test');
xlabel('Input Z (mm)');
ylabel('Output Z (mm)');
legend('Ideal response', 'Response (no load)', 'Response (with load)');
hold off;

%% Workspace XYZ Dual
stewie = StewartPlatform(152, 18.92, 103, 28.07, 73, 165);
x_valid = zeros(536264,1);
y_valid = zeros(536264,1);
z_valid = zeros(536264,1);
cnt=0;
for x = -200:2:200
    for y = -200:2:200
        for z = -250:5:250
            if stewie.set_position(0,0,0,x,y,z) == 1
                cnt = cnt+1;
                x_valid(cnt,1) = x;
                y_valid(cnt,1) = y;
                z_valid(cnt,1) = z;
                
            end
        end
    end
end
figure;
hold on;
k = boundary(x_valid(1:cnt,1),y_valid(1:cnt,1),z_valid(1:cnt,1),0.8);
trisurf(k,x_valid(1:cnt,1),y_valid(1:cnt,1),z_valid(1:cnt,1), 'Facecolor','cyan','Edgealpha',0.5,'Edgecolor',[0.3 0.7 0.7],'FaceAlpha',0.8);
axis equal;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Theoretical workspace of the fixed rotary actuated Stewart platform');
xlim([-200 200]);
ylim([-200 200]);
zlim([-250 250]);
hold off;

%% Workspace XYZ Upper
stewie = StewartPlatform(152, 18.92, 103, 28.07, 73, 165);
x_valid = zeros(268132,1);
y_valid = zeros(268132,1);
z_valid = zeros(268132,1);
cnt=0;
for x = -200:2:200
    for y = -200:2:200
        for z = -0:5:250
            if stewie.set_position(0,0,0,x,y,z) == 1
                cnt = cnt+1;
                x_valid(cnt,1) = x;
                y_valid(cnt,1) = y;
                z_valid(cnt,1) = z;
            end
        end
    end
end
figure;
hold on;
k = boundary(x_valid(1:cnt,1),y_valid(1:cnt,1),z_valid(1:cnt,1),0.8);
trisurf(k,x_valid(1:cnt,1),y_valid(1:cnt,1),z_valid(1:cnt,1), 'Facecolor','cyan','Edgealpha',0.5,'Edgecolor',[0.3 0.7 0.7],'FaceAlpha',0.8);
axis equal;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Theoretical upper workspace of the fixed rotary actuated Stewart platform');
xlim([-200 200]);
ylim([-200 200]);
zlim([0 250]);
hold off;

%% Workspace Roll Pitch Yaw 
stewie = StewartPlatform(152, 18.92, 103, 28.07, 73, 165);
roll_valid = zeros(100000,1);
pitch_valid = zeros(100000,1);
yaw_valid = zeros(100000,1);
cnt=0;
for roll = -90:1:90
    for pitch = -60:1:90
        for yaw = -90:1:90
            if stewie.set_position(roll,pitch,yaw,0,0,138)
                cnt = cnt+1;
                roll_valid(cnt,1) = roll;
                pitch_valid(cnt,1) = pitch;
                yaw_valid(cnt,1) = yaw;
                
            end
        end
    end
end
figure;
hold on;
k = boundary(roll_valid(1:cnt,1),pitch_valid(1:cnt,1),yaw_valid(1:cnt,1),0.8);
trisurf(k,roll_valid(1:cnt,1),pitch_valid(1:cnt,1),yaw_valid(1:cnt,1), 'Facecolor','cyan','Edgealpha',0.5,'Edgecolor',[0.3 0.7 0.7],'FaceAlpha',0.8);
axis equal;
xlabel('Roll (deg)');
ylabel('Pitch (deg)');
zlabel('Yaw (deg)');
title('Theoretical workspace of the fixed rotary actuated Stewart platform');
xlim([-90 90]);
ylim([-90 90]);
zlim([-90 90]);
hold off;