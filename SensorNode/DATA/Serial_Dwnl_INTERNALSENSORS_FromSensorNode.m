%%
close all
clear all

log_size = 68; %bytes
raw_file_n = ['Int_Sens_' strrep(datestr(datetime('now')),':','_') '.dat'];

instrreset;

ST = serial('COM3','BaudRate',921600);
ST.InputBufferSize = 2000;
ST.OutputBufferSize = 2000;

fopen(ST);
fileID = fopen(raw_file_n,'w');

fprintf(ST,'S');

pause(1)

fwrite(ST,'E','uchar');
pause(0.01);
flushinput(ST);

%wait start of tx
while ST.BytesAvailable < (ST.InputBufferSize/2)
    pause(0.001);
end


i=0;
j=1;
countt = 0;
while i==0
    pause(0.1);
    if ST.BytesAvailable == 0
        i=1;
        break;
    end
    [A,count] = fread(ST,ST.BytesAvailable,'uchar');
    countt = countt + count
    fwrite(fileID,A);
end

fclose(fileID);
fclose(ST);
clear A Index ST countt count

%%
% I need to remove this pattern since it is the comm overhead
oh = [13  10  85];
ohl = size(oh,2);

fileID = fopen(raw_file_n,'r');
raw = fread(fileID,Inf,'uchar');
fclose(fileID);

%remove first byte E
raw = raw(2:end);
%find pattern
Index  = strfind(raw', oh);
Index = flip(Index);
%number of elements to remove
nrem = size(Index,2) * ohl;
%need to remove the last CR+LF (\n\r)
nrem = nrem + 2;
%shift the array
for i = Index
    raw(i:end-ohl) = raw((i+ohl):end);
end
raw = uint8(raw(1:end-nrem));
%round size

%resave data
fileID = fopen(raw_file_n,'w');
fwrite(fileID,raw);
fclose(fileID);

clear raw oh ohl nrem Index
%%
%Import data into the struct
fileID = fopen(raw_file_n,'r');
[~,fil_size] = fread(fileID);
n_logs_f = floor(fil_size/log_size);
%return at the beginning
fseek(fileID,0,'bof');
for j=1:n_logs_f
    %/* °C */
	%float temp;
    log_temp(i) = fread(fileID,1,'single','ieee-le');
	%/* °C */
	%float temp_hum;
    log_temp_hum(i) = fread(fileID,1,'single','ieee-le');
	%/* % */
	%float hum;
    log_hum(i) = fread(fileID,1,'single','ieee-le');
	%/* Pascal - target 101325 Pa */
	%float pres;
    log_pres(i) = fread(fileID,1,'single','ieee-le');
	%/* x - y - z mG */
	%float magnetic_mG[3];
    log_magnetic_mG_x(i) = fread(fileID,1,'single','ieee-le');
    log_magnetic_mG_y(i) = fread(fileID,1,'single','ieee-le');
    log_magnetic_mG_z(i) = fread(fileID,1,'single','ieee-le');
	%/* x - y - z mG*/
	%float acceleration[3];
    log_acceleration_x(i) = fread(fileID,1,'single','ieee-le');
    log_acceleration_y(i) = fread(fileID,1,'single','ieee-le');
    log_acceleration_z(i) = fread(fileID,1,'single','ieee-le');
	%/* x - y -z mdps*/
	%float angular[3];
    log_angular_x(i) = fread(fileID,1,'single','ieee-le');
    log_angular_y(i) = fread(fileID,1,'single','ieee-le');
    log_angular_z(i) = fread(fileID,1,'single','ieee-le');
	%/* ADC0 V*/
	%float adc0;
    log_ADC_0(i) = fread(fileID,1,'single','ieee-le');
	%/* ADC1 V*/
	%float adc1;
    log_ADC_1(i) = fread(fileID,1,'single','ieee-le');
	%/* ADC2 V*/
	%float adc2;
    log_ADC_2(i) = fread(fileID,1,'single','ieee-le');
	%/* ADC3 V*/
	%float adc3;
    log_ADC_3(i) = fread(fileID,1,'single','ieee-le');
    
    i = i + 1;
end
fclose(fileID);

clear fileID fil_size

save(['Int_Sens_' strrep(datestr(datetime('now')),':','_')]);
