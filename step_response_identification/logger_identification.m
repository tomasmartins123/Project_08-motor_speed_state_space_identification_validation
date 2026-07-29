%% Project CRIA 08 - Identification Data Logger
clear; clc;

% ==========================================
% Configuration
% ==========================================
port = "COM3";               
baudRate = 115200;
outputFile = "step_response_identification_data.csv";
durationSec = 10;            % Test duration in seconds

% ==========================================
% Serial Connection & Logging
% ==========================================
try
    fprintf('Connecting to Arduino on port %s...\n', port);
    s = serialport(port, baudRate);
    configureTerminator(s, "LF");
    flush(s);

    fileID = fopen(outputFile, 'w');
    fprintf('Logging identification data to "%s" for %d seconds...\n\n', outputFile, durationSec);

    tStart = tic;
    linesCount = 0;

    while toc(tStart) < durationSec
        if s.NumBytesAvailable > 0
            dataLine = readline(s);
            dataLine = strtrim(dataLine);

            if ~strlength(dataLine) == 0
                fprintf(fileID, '%s\n', dataLine);
                disp(dataLine);
                linesCount = linesCount + 1;
            end
        end
    end

    fclose(fileID);
    clear s;
    fprintf('\nSuccess! %d lines recorded to "%s".\n', linesCount, outputFile);

catch ME
    if exist('fileID', 'var') && fileID ~= -1
        fclose(fileID);
    end
    if exist('s', 'var')
        clear s;
    end
    fprintf('\n[ERROR] Could not open port %s.\n', port);
    fprintf('Make sure the Arduino IDE Serial Monitor is CLOSED.\n');
    rethrow(ME);
end