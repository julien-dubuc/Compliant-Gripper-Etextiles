%% collect_gripper_patch_realtime.m
% Real-time acquisition for e-textile patch testing on the gripper.
%
% Expected Arduino serial line:
%   DATA,time_ms,Rx_ohm,Delta_R_ohm
% -------------------------------------------------------------------------

clear; clc;

%% ---------------- USER SETTINGS ----------------
disp("Available serial ports:");
disp(serialportlist("available"));

% Change this to your Arduino port.
port = "COM5";
baudRate = 115200;

% Use Inf to run until the figure is closed.
durationSec = Inf;

% Output file
timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
csvFileName = "gripper_patch_test_" + timestamp + ".csv";
csvPath = fullfile(pwd, csvFileName);

% Plot only the last N seconds
plotWindowSec = 60;

%% ---------------- CONNECT TO ARDUINO ----------------
fprintf("Connecting to %s at %d baud...\n", port, baudRate);

s = serialport(port, baudRate);
configureTerminator(s, "LF");
s.Timeout = 2;
flush(s);

pause(2);
flush(s);

disp("Connected.");
disp("1. Precondition the patch by closing/opening with the joystick.");
disp("2. Press the D4 button on the Arduino to Tare the resistance to zero.");
disp("3. Close the figure window to stop acquisition and save data.");

%% ---------------- LIVE PLOT ----------------
fig = figure("Name", "Gripper E-textile Patch Real-time", "NumberTitle", "off");

ax = axes(fig);
resLine = animatedline(ax, "MaximumNumPoints", 20000, "Color", "#D95319", "LineWidth", 1.5);
grid(ax, "on");
xlabel(ax, "Time (s)");
ylabel(ax, "\Delta Resistance (Ohm)");
title(ax, "Relative E-textile Resistance (Grasp Detection)");

%% ---------------- DATA STORAGE ----------------
time_s = zeros(0, 1);
arduino_ms = zeros(0, 1);
rx_ohm = zeros(0, 1);
delta_r = zeros(0, 1);
raw_line = strings(0, 1);

tStart = tic;

%% ---------------- ACQUISITION LOOP ----------------
while ishandle(fig)

    elapsed = toc(tStart);

    if isfinite(durationSec) && elapsed >= durationSec
        break;
    end

    while s.NumBytesAvailable > 0
        line = strtrim(readline(s));

        if strlength(line) == 0
            continue;
        end

        [ok, a_ms, rOhm, dR] = parseArduinoLine(line);

        if ~ok
            continue;
        end

        tNow = toc(tStart);

        time_s(end+1, 1) = tNow; %#ok<SAGROW>
        arduino_ms(end+1, 1) = a_ms; %#ok<SAGROW>
        rx_ohm(end+1, 1) = rOhm; %#ok<SAGROW>
        delta_r(end+1, 1) = dR; %#ok<SAGROW>
        raw_line(end+1, 1) = line; %#ok<SAGROW>

        % Plotting the delta_R for clear zero-baseline visualization
        addpoints(resLine, tNow, dR);

        xMin = max(0, tNow - plotWindowSec);
        xMax = max(plotWindowSec, tNow);
        xlim(ax, [xMin, xMax]);
    end

    drawnow limitrate;
end

%% ---------------- CLEANUP AND SAVE ----------------
clear s;  % closes serial port

time_s = time_s(:);
arduino_ms = arduino_ms(:);
rx_ohm = rx_ohm(:);
delta_r = delta_r(:);
raw_line = raw_line(:);

T = table(time_s, arduino_ms, rx_ohm, delta_r, raw_line, ...
    'VariableNames', {'time_s', 'arduino_ms', 'rx_ohm', 'delta_r', 'raw_line'});

writetable(T, csvPath);

fprintf("Acquisition stopped.\n");
fprintf("Saved %d samples to:\n%s\n", height(T), csvPath);

%% ---------------- LOCAL FUNCTION ----------------
function [ok, arduino_ms, rx_ohm, delta_r] = parseArduinoLine(line)
    ok = false;
    arduino_ms = NaN;
    rx_ohm = NaN;
    delta_r = NaN;

    line = string(strtrim(line));

    if ~startsWith(line, "DATA,")
        return;
    end

    parts = split(line, ",");

    % Expected: DATA,time_ms,Rx_ohm,Delta_R
    if numel(parts) >= 4
        arduino_ms = str2double(parts(2));
        rx_ohm = str2double(parts(3));
        delta_r = str2double(parts(4));

        ok = isfinite(arduino_ms) && isfinite(rx_ohm) && isfinite(delta_r);
    end
end