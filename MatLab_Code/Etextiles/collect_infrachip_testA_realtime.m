%% collect_infrachip_testA_realtime.m
% Real-time acquisition for the Arduino Test A-like strain routine.
%
% Expected Arduino serial line:
%   DATA,time_ms,force_N,Rx_ohm,state,step,target_strain_pct
%
% Example:
%   DATA,12345,1.234,38.800,STEP_HOLD,3,0.800
%
% Workflow:
%   1. Upload the Arduino sketch.
%   2. Close Arduino Serial Monitor.
%   3. Run this MATLAB script.
%   4. Press the physical D7 button to start the protocol.
%   5. Close the MATLAB figure to stop acquisition and save CSV.
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
csvFileName = "infrachip_testA_" + timestamp + ".csv";
csvPath = fullfile(pwd, csvFileName);

% Plot only the last N seconds
plotWindowSec = 120;

%% ---------------- CONNECT TO ARDUINO ----------------

fprintf("Connecting to %s at %d baud...\n", port, baudRate);

s = serialport(port, baudRate);
configureTerminator(s, "LF");
s.Timeout = 2;
flush(s);

% Give Arduino time to reset after serial connection, if it resets.
pause(2);
flush(s);

disp("Connected.");
disp("Press the D7 button on the Arduino setup to start the routine.");
disp("Close the figure window to stop acquisition and save data.");

%% ---------------- LIVE PLOTS ----------------

fig = figure("Name", "InfraChip Test A real-time acquisition", ...
             "NumberTitle", "off");

tiledlayout(fig, 2, 1);

ax1 = nexttile;
forceLine = animatedline(ax1, "MaximumNumPoints", 20000);
grid(ax1, "on");
xlabel(ax1, "Time (s)");
ylabel(ax1, "Force (N)");
title(ax1, "Load cell force");

ax2 = nexttile;
resLine = animatedline(ax2, "MaximumNumPoints", 20000);
grid(ax2, "on");
xlabel(ax2, "Time (s)");
ylabel(ax2, "Resistance (Ohm)");
title(ax2, "E-textile resistance");

%% ---------------- DATA STORAGE ----------------

time_s = zeros(0, 1);
arduino_ms = zeros(0, 1);
force_N = zeros(0, 1);
rx_ohm = zeros(0, 1);
state = strings(0, 1);
step = zeros(0, 1);
target_strain_pct = zeros(0, 1);
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

        [ok, a_ms, fN, rOhm, st, stepIdx, targetStrain] = parseArduinoLine(line);

        if ~ok
            % Uncomment this line if you want to inspect ignored text.
            % fprintf("Ignored: %s\n", line);
            continue;
        end

        tNow = toc(tStart);

        time_s(end+1, 1) = tNow; %#ok<SAGROW>
        arduino_ms(end+1, 1) = a_ms; %#ok<SAGROW>
        force_N(end+1, 1) = fN; %#ok<SAGROW>
        rx_ohm(end+1, 1) = rOhm; %#ok<SAGROW>
        state(end+1, 1) = st; %#ok<SAGROW>
        step(end+1, 1) = stepIdx; %#ok<SAGROW>
        target_strain_pct(end+1, 1) = targetStrain; %#ok<SAGROW>
        raw_line(end+1, 1) = line; %#ok<SAGROW>

        addpoints(forceLine, tNow, fN);
        addpoints(resLine, tNow, rOhm);

        xMin = max(0, tNow - plotWindowSec);
        xMax = max(plotWindowSec, tNow);
        xlim(ax1, [xMin, xMax]);
        xlim(ax2, [xMin, xMax]);
    end

    drawnow limitrate;
end

%% ---------------- CLEANUP AND SAVE ----------------

clear s;  % closes serial port

% Ensure column vectors
time_s = time_s(:);
arduino_ms = arduino_ms(:);
force_N = force_N(:);
rx_ohm = rx_ohm(:);
state = state(:);
step = step(:);
target_strain_pct = target_strain_pct(:);
raw_line = raw_line(:);

if isempty(time_s)
    warning("No valid DATA lines were recorded. Check Arduino output format.");
end

T = table(time_s, arduino_ms, force_N, rx_ohm, state, step, target_strain_pct, raw_line, ...
    'VariableNames', {'time_s', 'arduino_ms', 'force_N', 'rx_ohm', ...
                      'state', 'step', 'target_strain_pct', 'raw_line'});

writetable(T, csvPath);

fprintf("Acquisition stopped.\n");
fprintf("Saved %d samples to:\n%s\n", height(T), csvPath);

%% ---------------- LOCAL FUNCTION ----------------

function [ok, arduino_ms, force_N, rx_ohm, state, step, target_strain_pct] = parseArduinoLine(line)

    ok = false;
    arduino_ms = NaN;
    force_N = NaN;
    rx_ohm = NaN;
    state = "";
    step = NaN;
    target_strain_pct = NaN;

    line = string(strtrim(line));

    if ~startsWith(line, "DATA,")
        return;
    end

    parts = split(line, ",");

    % Expected:
    % DATA,time_ms,force_N,Rx_ohm,state,step,target_strain_pct
    if numel(parts) >= 7
        arduino_ms = str2double(parts(2));
        force_N = str2double(parts(3));
        rx_ohm = str2double(parts(4));
        state = string(parts(5));
        step = str2double(parts(6));
        target_strain_pct = str2double(parts(7));

        ok = isfinite(arduino_ms) && ...
             isfinite(force_N) && ...
             isfinite(rx_ohm) && ...
             isfinite(step) && ...
             isfinite(target_strain_pct);
    end
end
