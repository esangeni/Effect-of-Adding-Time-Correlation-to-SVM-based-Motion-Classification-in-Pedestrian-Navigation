clc
clear
close all

% datasetFolder = [pwd '\dataset\'];
% datasetFolder = '\Google Drive\My Career\UCI\Research\NIST\uNavChip\navigation algorithm\dataset\'; % main working folder storing all navigation information
datasetFolder = ['G:\Shared drives\NIST-data\']


dataFolder = '2022_09_02';

FIGURES = findall(0,'type','figure'); % For clearing all figures.
for i=1:length(FIGURES)
    figure(i)
    clf(FIGURES(i));
    %hold on
end
filpath  = [datasetFolder dataFolder];
savepath = [datasetFolder dataFolder '\']; %Saved Directory
mode = 'dat'; %'bin' 'dat' 'csv'
n = dir([filpath '\*.' mode]); % Change me
for j =1:size(n)
    fname = n(j).name(1:end-4);
    %     if strcmp(fname,'exp6300')
    %     if 1
    filename = [filpath '\' fname ];
    if ~isfile([filename '_VN.mat']) || 1
        
        g  = 9.817269086191379;
        %f0 = 492; %in Hz used by ADIS % Change me
        %f0 = 472; %in Hz used by ADIS
        f0 = 800;%1/2e-3; %in Hz used in Flex platform
        %load('H:\My Drive\Documents\PhD-research\NIST\Experimental Data\RecordingFromADIS\ADIS16485-ARW-DataAnalysisExample.mat');Y_GYRO16bit = Sina;
        
        
        datafile = [filename '.' mode];
        if mode == 'dat' %this is for VN sensor
            data = importVNfile(datafile,'comp');
            IMUdata = 'comp' ; %'comp' 'normal'
            switch IMUdata
                case 'normal'
                    u(1,:) = data.AccelerationXms2VectorNavVn200StandardRugged0100028726/g; % X acc, in g [m/s^2]
                    u(2,:) = data.AccelerationYms2VectorNavVn200StandardRugged0100028726/g; % Y acc
                    u(3,:) = data.AccelerationZms2VectorNavVn200StandardRugged0100028726/g; % Z acc
                    u(4,:) = data.AngularRateXdegssVectorNavVn200StandardRugged0100028726; % X gyr, in deg/sec
                    u(5,:) = data.AngularRateYdegssVectorNavVn200StandardRugged0100028726; % Y gyr
                    u(6,:) = data.AngularRateZdegssVectorNavVn200StandardRugged0100028726; % Z gyr
                    if data.MagneticXnaVectorNavVn200StandardRugged0100028726(1) ~= ""
                        if isa(data.MagneticXnaVectorNavVn200StandardRugged0100028726(1),'string')
                            u(12,:) = str2double(data.MagneticXnaVectorNavVn200StandardRugged0100028726); % mag x, uT
                            u(13,:) = str2double(data.MagneticYnaVectorNavVn200StandardRugged0100028726); % mag y, uT
                            u(14,:) = str2double(data.MagneticZnaVectorNavVn200StandardRugged0100028726); % mag z, uT
                        else
                            u(12,:) = data.MagneticXnaVectorNavVn200StandardRugged0100028726; % mag x, uT
                            u(13,:) = data.MagneticYnaVectorNavVn200StandardRugged0100028726; % mag y, uT
                            u(14,:) = data.MagneticZnaVectorNavVn200StandardRugged0100028726; % mag z, uT
                        end
                    end
                case 'comp'
                    % re assign the dataset
                    if 0
                        data.UncompensatedAccelerationXms2VectorNavVn200StandardRugged010002 = str2double(data.QuaternionXnaVectorNavVn200StandardRugged0100028726); % X acc, in g [m/s^2]
                        data.UncompensatedAccelerationYms2VectorNavVn200StandardRugged010002 = str2double(data.QuaternionYnaVectorNavVn200StandardRugged0100028726); % Y acc
                        data.UncompensatedAccelerationZms2VectorNavVn200StandardRugged010002 = str2double(data.QuaternionZnaVectorNavVn200StandardRugged0100028726); % Z acc
                        data.UncompensatedAngularRateXdegssVectorNavVn200StandardRugged01000 = str2double(data.Dcm02naVectorNavVn200StandardRugged0100028726); % X gyr, in deg/sec
                        data.UncompensatedAngularRateYdegssVectorNavVn200StandardRugged01000 = str2double(data.Dcm10naVectorNavVn200StandardRugged0100028726); % Y gyr
                        data.UncompensatedAngularRateZdegssVectorNavVn200StandardRugged01000 = str2double(data.Dcm11naVectorNavVn200StandardRugged0100028726); % Z gyr
                        data.UncompensatedMagneticXnaVectorNavVn200StandardRugged0100028726 = (data.QuaternionWnaVectorNavVn200StandardRugged0100028726); % mag x, uT
                        data.UncompensatedMagneticYnaVectorNavVn200StandardRugged0100028726 = (data.Dcm00naVectorNavVn200StandardRugged0100028726); % mag y, uT
                        data.UncompensatedMagneticZnaVectorNavVn200StandardRugged0100028726 = (data.Dcm01naVectorNavVn200StandardRugged0100028726); % mag z, uT
                        data.PressurekPaVectorNavVn200StandardRugged0100028726 = data.Dcm01naVectorNavVn200StandardRugged0100028726;
                    end
                    
                    u(1,:) = data.UncompensatedAccelerationXms2VectorNavVn200StandardRugged010002/g; % X acc, in g [m/s^2]
                    u(2,:) = data.UncompensatedAccelerationYms2VectorNavVn200StandardRugged010002/g; % Y acc
                    u(3,:) = data.UncompensatedAccelerationZms2VectorNavVn200StandardRugged010002/g; % Z acc
                    u(4,:) = data.UncompensatedAngularRateXdegssVectorNavVn200StandardRugged01000; % X gyr, in deg/sec
                    u(5,:) = data.UncompensatedAngularRateYdegssVectorNavVn200StandardRugged01000; % Y gyr
                    u(6,:) = data.UncompensatedAngularRateZdegssVectorNavVn200StandardRugged01000; % Z gyr
                    if data.UncompensatedMagneticXnaVectorNavVn200StandardRugged0100028726(1) ~= ""
                        if isa(data.UncompensatedMagneticXnaVectorNavVn200StandardRugged0100028726(1),'string')
                            u(12,:) = str2double(data.UncompensatedMagneticXnaVectorNavVn200StandardRugged0100028726); % mag x, uT
                            u(13,:) = str2double(data.UncompensatedMagneticYnaVectorNavVn200StandardRugged0100028726); % mag y, uT
                            u(14,:) = str2double(data.UncompensatedMagneticZnaVectorNavVn200StandardRugged0100028726); % mag z, uT
                        else
                            u(12,:) = data.UncompensatedMagneticXnaVectorNavVn200StandardRugged0100028726; % mag x, uT
                            u(13,:) = data.UncompensatedMagneticYnaVectorNavVn200StandardRugged0100028726; % mag y, uT
                            u(14,:) = data.UncompensatedMagneticZnaVectorNavVn200StandardRugged0100028726; % mag z, uT
                        end
                    end
                    %                 if data.TemperatureCVectorNavVn200StandardRugged0100028726(1) ~= ""
                    %                     if isa(data.TemperatureCVectorNavVn200StandardRugged0100028726(1),'string')
                    %                         u(15,:) = str2double(data.TemperatureCVectorNavVn200StandardRugged0100028726); % temperature, C
                    %                     else
                    %                         u(15,:) = data.TemperatureCVectorNavVn200StandardRugged0100028726; % temperature, C
                    %                     end
                    %                 end
                    %                 if data.PressurekPaVectorNavVn200StandardRugged0100028726(1) ~= ""
                    %                     if isa(data.PressurekPaVectorNavVn200StandardRugged0100028726(1),'string')
                    %                         u(10,:) = str2double(data.PressurekPaVectorNavVn200StandardRugged0100028726); % pressure, kPa
                    %                     else
                    %                         u(10,:) = data.PressurekPaVectorNavVn200StandardRugged0100028726; % pressure, kPa
                    %                     end
                    %                 end
            end
%                     time = data.Timestamp - data.Timestamp(1);
            time = 0:1/f0:1/f0*(length(data.Timestamp)-1);
            u(7,:) = u(1,:)*0; % flag gyr
            u(8,:) = time;
            u(9,:) = u(1,:)*0; % Debug var
%                   u(11,:) = [diff(time)' time(end)-time(end-1)]; % sampling period, s
            u(11,:) = 1/f0; % sampling period, s
            if data.PressurekPaVectorNavVn200StandardRugged0100028726(1)~= ""
                if isa(data.PressurekPaVectorNavVn200StandardRugged0100028726(1),'string')
                    u(10,:) = str2double(data.PressurekPaVectorNavVn200StandardRugged0100028726)*1e3; % pressure sensor, Pa
                    %                 u(11,:) = [diff(time)' time(end)-time(end-1)]; % sampling period, s
                    %                 u(11,:) = 1/f0; % sampling period, s
                    u(15,:) = str2double(data.TemperatureCVectorNavVn200StandardRugged0100028726); % temperature, degree C
                else
                    u(10,:) = data.PressurekPaVectorNavVn200StandardRugged0100028726*1e3; % pressure sensor, Pa
                    %                 u(11,:) = [diff(time)' time(end)-time(end-1)]; % sampling period, s
                    %                 u(11,:) = 1/f0; % sampling period, s
                    u(15,:) = data.TemperatureCVectorNavVn200StandardRugged0100028726; % temperature, degree C
                end
            end
            clear data
        else
            disp('".dat" file not found.')
        end
        
        if exist('u2','var')
            fname_sav = [savepath fname '_VN_L.mat'];
            u = u1;
            save(fname_sav,'u')  % Normal recording.
            disp(fname_sav)
            fname_sav = [savepath fname '_VN_R.mat'];
            u = u2;
            save(fname_sav,'u')  % Normal recording.
            disp(fname_sav)
            clear u u1 u2
        else
            %         fname_sav = [savepath fname '_VN.mat'];
            fname_sav = [savepath fname '_VN.mat'];
            save(fname_sav,'u')  % Normal recording.
            disp(fname_sav)
            clear u1
        end
        clear u
    end
end

function data = importVNfile(filename,mode, startRow, endRow)
%IMPORTFILE Import numeric data from a text file as a matrix.
%   TESTNUME1 = IMPORTFILE(FILENAME) Reads data from text file FILENAME for
%   the default selection.
%
%   TESTNUME1 = IMPORTFILE(FILENAME, STARTROW, ENDROW) Reads data from rows
%   STARTROW through ENDROW of text file FILENAME.
%
% Example:
%   testnume1 = importfile('testnume1.dat', 2, 1997);
%
%    See also TEXTSCAN.

% Auto-generated by MATLAB on 2018/07/11 10:58:54

%% Initialize variables.
delimiter = '\t';
if nargin<=3
    startRow = 2;
    endRow = inf;
end

%% Format for each line of text:
%   column1: double (%f)
%	column2: text (%q)
%   column3: text (%q)
%	column4: text (%q)
%   column5: text (%q)
%	column6: text (%q)
%   column7: text (%q)
%	column8: text (%q)
%   column9: text (%q)
%	column10: text (%q)
%   column11: text (%q)
%	column12: text (%q)
%   column13: text (%q)
%	column14: text (%q)
%   column15: text (%q)
%	column16: text (%q)
%   column17: text (%q)
%	column18: text (%q)
%   column19: text (%q)
%	column20: text (%q)
%   column21: text (%q)
%	column22: text (%q)
%   column23: text (%q)
%	column24: double (%f) AccelerationXms2
%   column25: double (%f) AccelerationYms2
%	column26: double (%f) AccelerationZms2
%   column27: text (%q)
%	column28: text (%q)
%   column29: text (%q)
%	column30: double (%f) AngularRateXdegss
%   column31: double (%f) AngularRateYdegss
%	column32: double (%f) AngularRateZdegss
%   column33: text (%f)  UncompensatedAccelerationXms2
%	column34: text (%f)  UncompensatedAccelerationYms2
%   column35: text (%f)  UncompensatedAccelerationZms2
%	column36: text (%q)  
%   column37: text (%q)
%	column38: text (%q)
%   column39: text (%f)  UncompensatedAngularRateXdegss
%	column40: text (%f)  UncompensatedAngularRateYdegss
%   column41: text (%f)  UncompensatedAngularRateZdegss
%	column42: text (%q)
%   column43: text (%q)
%	column44: text (%q)
%   column45: text (%q)
%	column46: text (%q)
%   column47: text (%q)
%	column48: text (%q)
%   column49: text (%q)
%	column50: text (%q)
%   column51: text (%q)
%	column52: text (%q)
%   column53: text (%q)
%	column54: text (%q)
%   column55: text (%q)
%	column56: text (%q)
%   column57: text (%q)
%	column58: text (%q)
%   column59: text (%q)
%	column60: text (%q)
%   column61: text (%q)
%	column62: text (%q)
%   column63: text (%q)
%	column64: text (%q)
%   column65: text (%q)
%	column66: text (%q)
%   column67: text (%q)
%	column68: text (%q)
%   column69: text (%q)
%	column70: text (%q)
%   column71: text (%q)
%	column72: text (%q)
%   column73: text (%q)
%	column74: text (%q)
%   column75: text (%q)
%	column76: text (%q)
%   column77: text (%q)
%	column78: text (%q)
%   column79: text (%q)
%	column80: text (%q)
%   column81: text (%q)
%	column82: text (%q)
%   column83: text (%q)
%	column84: text (%q)
%   column85: text (%q)
%	column86: text (%q)
%   column87: text (%q)
%	column88: text (%q)
%   column89: text (%q)
%	column90: text (%q)
%   column91: text (%q)
%	column92: text (%q)
%   column93: text (%q)
%	column94: text (%q)
%   column95: text (%q)
%	column96: text (%q)
%   column97: text (%q)
%	column98: text (%q)
%   column99: text (%q)
%	column100: text (%q)
%   column101: text (%q)
%	column102: text (%q)
%   column103: text (%q)
%	column104: text (%q)
%   column105: text (%q)
%	column106: text (%q)
%   column107: text (%q)
%	column108: text (%q)
%   column109: text (%q)
%	column110: text (%q)
%   column111: text (%q)
%	column112: text (%q)
%   column113: text (%q)
%	column114: text (%q)
%   column115: text (%q)
%	column116: text (%q)
%   column117: text (%q)
%	column118: text (%q)
%   column119: text (%q)
%	column120: text (%q)
%   column121: text (%q)
%	column122: text (%q)
%   column123: text (%q)
%	column124: text (%q)
%   column125: text (%q)
%	column126: text (%q)
%   column127: text (%q)
%	column128: text (%q)
%   column129: text (%q)
%	column130: text (%q)
% For more information, see the TEXTSCAN documentation.
switch mode
    case 'normal'
        formatSpec = '%f%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%f%f%f%q%q%q%f%f%f%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%[^\n\r]';
    case 'comp'
        formatSpec = '%f%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%f%f%f%q%q%q%f%f%f%f%f%f%q%q%q%f%f%f%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%[^\n\r]';
end

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines', startRow(1)-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
for block=2:length(startRow)
    frewind(fileID);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines', startRow(block)-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Create output variable
data = table(dataArray{1:end-1}, 'VariableNames', {'Timestamp','AttitudeYawPitchRollYawdegsVectorNavVn200StandardRugged01000287',...
    'AttitudeYawPitchRollPitchdegsVectorNavVn200StandardRugged010002','AttitudeYawPitchRollRolldegsVectorNavVn200StandardRugged0100028',...    
    'YawPitchRollYawdegsVectorNavVn200StandardRugged0100028726','YawPitchRollPitchdegsVectorNavVn200StandardRugged0100028726',...
    'YawPitchRollRolldegsVectorNavVn200StandardRugged0100028726','QuaternionXnaVectorNavVn200StandardRugged0100028726',...
    'QuaternionYnaVectorNavVn200StandardRugged0100028726','QuaternionZnaVectorNavVn200StandardRugged0100028726',...
    'QuaternionWnaVectorNavVn200StandardRugged0100028726','Dcm00naVectorNavVn200StandardRugged0100028726',...
    'Dcm01naVectorNavVn200StandardRugged0100028726','Dcm02naVectorNavVn200StandardRugged0100028726','Dcm10naVectorNavVn200StandardRugged0100028726',...
    'Dcm11naVectorNavVn200StandardRugged0100028726','Dcm12naVectorNavVn200StandardRugged0100028726',...
    'Dcm20naVectorNavVn200StandardRugged0100028726','Dcm21naVectorNavVn200StandardRugged0100028726',...
    'Dcm22naVectorNavVn200StandardRugged0100028726','YawPitchRollUncertaintyXdegVectorNavVn200StandardRugged01000287',...
    'YawPitchRollUncertaintyYdegVectorNavVn200StandardRugged01000287','YawPitchRollUncertaintyZdegVectorNavVn200StandardRugged01000287',...
    'AccelerationXms2VectorNavVn200StandardRugged0100028726',...
    'AccelerationYms2VectorNavVn200StandardRugged0100028726',...
    'AccelerationZms2VectorNavVn200StandardRugged0100028726',...
    'MagneticXnaVectorNavVn200StandardRugged0100028726','MagneticYnaVectorNavVn200StandardRugged0100028726','MagneticZnaVectorNavVn200StandardRugged0100028726',...
    'AngularRateXdegssVectorNavVn200StandardRugged0100028726',...
    'AngularRateYdegssVectorNavVn200StandardRugged0100028726',...
    'AngularRateZdegssVectorNavVn200StandardRugged0100028726',...
    'UncompensatedAccelerationXms2VectorNavVn200StandardRugged010002',...
    'UncompensatedAccelerationYms2VectorNavVn200StandardRugged010002',...
    'UncompensatedAccelerationZms2VectorNavVn200StandardRugged010002',...
    'UncompensatedMagneticXnaVectorNavVn200StandardRugged0100028726','UncompensatedMagneticYnaVectorNavVn200StandardRugged0100028726','UncompensatedMagneticZnaVectorNavVn200StandardRugged0100028726',...
    'UncompensatedAngularRateXdegssVectorNavVn200StandardRugged01000',...
    'UncompensatedAngularRateYdegssVectorNavVn200StandardRugged01000',...
    'UncompensatedAngularRateZdegssVectorNavVn200StandardRugged01000',...
    'TemperatureCVectorNavVn200StandardRugged0100028726','PressurekPaVectorNavVn200StandardRugged0100028726','PositionLatitudenaVectorNavVn200StandardRugged0100028726',...
    'PositionLongitudenaVectorNavVn200StandardRugged0100028726','PositionAltitudenaVectorNavVn200StandardRugged0100028726',...
    'EstimatedPositionLlaLatitudedegdegmVectorNavVn200StandardRugged','EstimatedPositionLlaLongitudedegdegmVectorNavVn200StandardRugge',...
    'EstimatedPositionLlaAltitudedegdegmVectorNavVn200StandardRugged','EstimatedPositionEcefXmVectorNavVn200StandardRugged0100028726',...
    'EstimatedPositionEcefYmVectorNavVn200StandardRugged0100028726','EstimatedPositionEcefZmVectorNavVn200StandardRugged0100028726',...
    'EstimatedVelocityBodyXmsVectorNavVn200StandardRugged0100028726','EstimatedVelocityBodyYmsVectorNavVn200StandardRugged0100028726',...
    'EstimatedVelocityBodyZmsVectorNavVn200StandardRugged0100028726','EstimatedVelocityNedXmsVectorNavVn200StandardRugged0100028726',...
    'EstimatedVelocityNedYmsVectorNavVn200StandardRugged0100028726','EstimatedVelocityNedZmsVectorNavVn200StandardRugged0100028726',...
    'EstimatedVelocityEcefXmsVectorNavVn200StandardRugged0100028726','EstimatedVelocityEcefYmsVectorNavVn200StandardRugged0100028726',...
    'EstimatedVelocityEcefZmsVectorNavVn200StandardRugged0100028726','EstimatedPositionUncertaintymVectorNavVn200StandardRugged010002',...
    'EstimatedVelocityUncertaintymVectorNavVn200StandardRugged010002','GpsTowsVectorNavVn200StandardRugged0100028726',...
    'GpsTowNsnanosecondsVectorNavVn200StandardRugged0100028726','GpsWeekweekVectorNavVn200StandardRugged0100028726',...
    'GpsTimestampLocalTimenaVectorNavVn200StandardRugged0100028726','GpsPositionLlaLatitudedegdegmVectorNavVn200StandardRugged010002',...
    'GpsPositionLlaLongitudedegdegmVectorNavVn200StandardRugged01000','GpsPositionLlaAltitudedegdegmVectorNavVn200StandardRugged010002',...
    'GpsPositionEcefXmVectorNavVn200StandardRugged0100028726','GpsPositionEcefYmVectorNavVn200StandardRugged0100028726',...
    'GpsPositionZmVectorNavVn200StandardRugged0100028726','GpsVelocityNedXmsVectorNavVn200StandardRugged0100028726',...
    'GpsVelocityNedYmsVectorNavVn200StandardRugged0100028726','GpsVelocityNedZmsVectorNavVn200StandardRugged0100028726',...
    'GpsVelocityEcefXmsVectorNavVn200StandardRugged0100028726','GpsVelocityEcefYmsVectorNavVn200StandardRugged0100028726',...
    'GpsVelocityEcefZmsVectorNavVn200StandardRugged0100028726','GpsPositionUncertaintyXmVectorNavVn200StandardRugged0100028726',...
    'GpsPositionUncertaintyYmVectorNavVn200StandardRugged0100028726','GpsPositionUncertaintyZmVectorNavVn200StandardRugged0100028726',...
    'GpsVelocityUncertaintymsVectorNavVn200StandardRugged0100028726','GpsTimeUncertaintysecVectorNavVn200StandardRugged0100028726',...
    'GpsFixnaVectorNavVn200StandardRugged0100028726','NumberOfSatellitesnaVectorNavVn200StandardRugged0100028726','ImuStatusnaVectorNavVn200StandardRugged0100028726',...
    'VpeStatusnaVectorNavVn200StandardRugged0100028726','InsFilterStatusModenaVectorNavVn200StandardRugged0100028726',...
    'InsFilterStatusGpsFixnaVectorNavVn200StandardRugged0100028726','InsFilterStatusErrornaVectorNavVn200StandardRugged0100028726',...
    'SensorSaturationnaVectorNavVn200StandardRugged0100028726','TimeStartupnanosecondsVectorNavVn200StandardRugged0100028726',...
    'TimeSyncInnanosecondsVectorNavVn200StandardRugged0100028726','SyncInCountnaVectorNavVn200StandardRugged0100028726',...
    'DeltaTimenaVectorNavVn200StandardRugged0100028726','DeltaThetaXdegsVectorNavVn200StandardRugged0100028726','DeltaThetaYdegsVectorNavVn200StandardRugged0100028726',...
    'DeltaThetaZdegsVectorNavVn200StandardRugged0100028726','DeltaVelocityXmsVectorNavVn200StandardRugged0100028726',...
    'DeltaVelocityYmsVectorNavVn200StandardRugged0100028726','DeltaVelocityZmsVectorNavVn200StandardRugged0100028726',...
    'TimeGpsnanosecondsVectorNavVn200StandardRugged0100028726','TimeGpsPpsnanosecondsVectorNavVn200StandardRugged0100028726',...
    'TimeUtcnaVectorNavVn200StandardRugged0100028726','AccelerationNedXms2VectorNavVn200StandardRugged0100028726',...
    'AccelerationNedYms2VectorNavVn200StandardRugged0100028726','AccelerationNedZms2VectorNavVn200StandardRugged0100028726',...
    'LinearAccelerationBodyXms2VectorNavVn200StandardRugged010002872','LinearAccelerationBodyYms2VectorNavVn200StandardRugged010002872',...
    'LinearAccelerationBodyZms2VectorNavVn200StandardRugged010002872','LinearAccelerationNedXms2VectorNavVn200StandardRugged0100028726',...
    'LinearAccelerationNedYms2VectorNavVn200StandardRugged0100028726','LinearAccelerationNedZms2VectorNavVn200StandardRugged0100028726',...
    'AccelerationEcefXms2VectorNavVn200StandardRugged0100028726','AccelerationEcefYms2VectorNavVn200StandardRugged0100028726',...
    'AccelerationEcefZms2VectorNavVn200StandardRugged0100028726','LinearAccelerationEcefXms2VectorNavVn200StandardRugged010002872',...
    'LinearAccelerationEcefYms2VectorNavVn200StandardRugged010002872','LinearAccelerationEcefZms2VectorNavVn200StandardRugged010002872',...
    'MagneticNedXnaVectorNavVn200StandardRugged0100028726','MagneticNedYnaVectorNavVn200StandardRugged0100028726','MagneticNedZnaVectorNavVn200StandardRugged0100028726',...
    'MagneticEcefXnaVectorNavVn200StandardRugged0100028726','MagneticEcefYnaVectorNavVn200StandardRugged0100028726','MagneticEcefZnaVectorNavVn200StandardRugged0100028726',...
    'GpsPositionAccuracyEcefXmVectorNavVn200StandardRugged0100028726','GpsPositionAccuracyEcefYmVectorNavVn200StandardRugged0100028726',...
    'GpsPositionAccuracyEcefZmVectorNavVn200StandardRugged0100028726','EstimatedAttitudeUncertaintydegVectorNavVn200StandardRugged0100'});

end