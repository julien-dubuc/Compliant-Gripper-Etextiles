%% collect_infrachip_testB_caliper_realtime.m
% Real-time acquisition for InfraChip Test B force-cycling routine.
%
% Arduino line format:
%   DATA,time_ms,force_N,Rx_ohm,state,cycle,target_force_N,measure_needed,measure_ready,hold_elapsed_s
%
% Example:
%   DATA,12345,1.503,38.800,LOW_HOLD,7,1.500,0,0,12.30
%
% Workflow:
%   1. Upload the Arduino Test B caliper sketch.
%   2. Close Arduino Serial Monitor.
%   3. Run this MATLAB script.
%   4. Press D7 on the Arduino setup to start the routine.
%   5. When D8 is HIGH, take the caliper length measurement.
%   6. Close the figure window to stop acquisition and save CSV.

clear; clc;

%% ---------------- USER SETTINGS ----------------

disp("Available serial ports:");
disp(serialportlist("available"));

port = "COM5";       % Change this to your Arduino port.
baudRate = 115200;

durationSec = Inf;

timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
csvFileName = "infrachip_testB_caliper_" + timestamp + ".csv";
csvPath = fullfile(pwd, csvFileName);

plotWindowSec = 120;

%% ---------------- CONNECT TO ARDUINO ----------------

fprintf("Connecting to %s at %d baud...\n", port, baudRate);

s = serialport(port, baudRate);
configureTerminator(s, "LF");
s.Timeout = 2;
flush(s);

pause(2);
flush(s);

disp("Connected.");
disp("Press D7 on the Arduino setup to start the routine.");
disp("When D8 is HIGH, take the caliper length measurement.");
disp("Close the figure window to stop acquisition and save CSV.");

%% ---------------- LIVE PLOTS ----------------

fig = figure("Name", "InfraChip Test B caliper acquisition", ...
             "NumberTitle", "off");

tiledlayout(fig, 2, 1);

ax1 = nexttile;
forceLine = animatedline(ax1, "MaximumNumPoints", 30000);
grid(ax1, "on");
xlabel(ax1, "Time (s)");
ylabel(ax1, "Force (N)");
title(ax1, "Load cell force");

ax2 = nexttile;
resLine = animatedline(ax2, "MaximumNumPoints", 30000);
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
cycle = zeros(0, 1);
target_force_N = zeros(0, 1);
measure_needed = zeros(0, 1);
measure_ready = zeros(0, 1);
hold_elapsed_s = zeros(0, 1);
raw_line = strings(0, 1);

lastShownCueKey = "";

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

        [ok, a_ms, fN, rOhm, st, cyc, targetN, measNeed, measReady, holdElapsed] = parseArduinoLine(line);

        if ~ok
            continue;
        end

        tNow = toc(tStart);

        time_s(end+1, 1) = tNow; %#ok<SAGROW>
        arduino_ms(end+1, 1) = a_ms; %#ok<SAGROW>
        force_N(end+1, 1) = fN; %#ok<SAGROW>
        rx_ohm(end+1, 1) = rOhm; %#ok<SAGROW>
        state(end+1, 1) = st; %#ok<SAGROW>
        cycle(end+1, 1) = cyc; %#ok<SAGROW>
        target_force_N(end+1, 1) = targetN; %#ok<SAGROW>
        measure_needed(end+1, 1) = measNeed; %#ok<SAGROW>
        measure_ready(end+1, 1) = measReady; %#ok<SAGROW>
        hold_elapsed_s(end+1, 1) = holdElapsed; %#ok<SAGROW>
        raw_line(end+1, 1) = line; %#ok<SAGROW>

        addpoints(forceLine, tNow, fN);
        addpoints(resLine, tNow, rOhm);

        if measReady == 1
            cueKey = sprintf("%s_cycle%d_force%.1f", st, cyc, targetN);
            if cueKey ~= lastShownCueKey
                fprintf("CALIPER MEASUREMENT READY: state=%s, cycle=%d, target=%.1f N\n", ...
                    st, cyc, targetN);
                lastShownCueKey = cueKey;
            end
        end

        xMin = max(0, tNow - plotWindowSec);
        xMax = max(plotWindowSec, tNow);
        xlim(ax1, [xMin, xMax]);
        xlim(ax2, [xMin, xMax]);
    end

    drawnow limitrate;
end

%% ---------------- CLEANUP AND SAVE ----------------

clear s;

T = table(time_s(:), arduino_ms(:), force_N(:), rx_ohm(:), ...
          state(:), cycle(:), target_force_N(:), ...
          measure_needed(:), measure_ready(:), hold_elapsed_s(:), raw_line(:), ...
    'VariableNames', {'time_s', 'arduino_ms', 'force_N', 'rx_ohm', ...
                      'state', 'cycle', 'target_force_N', ...
                      'measure_needed', 'measure_ready', 'hold_elapsed_s', 'raw_line'});

writetable(T, csvPath);

fprintf("Acquisition stopped.\n");
fprintf("Saved %d samples to:\n%s\n", height(T), csvPath);

%% ---------------- LOCAL FUNCTION ----------------

function [ok, arduino_ms, force_N, rx_ohm, state, cycle, target_force_N, measure_needed, measure_ready, hold_elapsed_s] = parseArduinoLine(line)

    ok = false;
    arduino_ms = NaN;
    force_N = NaN;
    rx_ohm = NaN;
    state = "";
    cycle = NaN;
    target_force_N = NaN;
    measure_needed = NaN;
    measure_ready = NaN;
    hold_elapsed_s = NaN;

    line = string(strtrim(line));

    if ~startsWith(line, "DATA,")
        return;
    end

    parts = split(line, ",");

    if numel(parts) >= 10
        arduino_ms = str2double(parts(2));
        force_N = str2double(parts(3));
        rx_ohm = str2double(parts(4));
        state = string(parts(5));
        cycle = str2double(parts(6));
        target_force_N = str2double(parts(7));
        measure_needed = str2double(parts(8));
        measure_ready = str2double(parts(9));
        hold_elapsed_s = str2double(parts(10));

        ok = isfinite(arduino_ms) && ...
             isfinite(force_N) && ...
             isfinite(rx_ohm) && ...
             isfinite(cycle) && ...
             isfinite(target_force_N) && ...
             isfinite(measure_needed) && ...
             isfinite(measure_ready) && ...
             isfinite(hold_elapsed_s);
    end
end
