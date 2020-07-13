%%
close all
clear all

instrreset;
raw_file_n = ['Audio_' strrep(datestr(datetime('now')),':','_') '.dat'];

ST = serial('COM3','BaudRate',921600);
ST.InputBufferSize = 200000;
ST.OutputBufferSize = 200000;

fopen(ST);
fileID = fopen(raw_file_n,'w');

fprintf(ST,'S');

pause(1)

fwrite(ST,'E','uchar');
pause(0.01);
flushinput(ST);

%[tlineA,count] = fgetl(SA)
%[tlineT,count] = fgetl(ST)

%wait start of tx
while ST.BytesAvailable < (ST.InputBufferSize/20)
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

% cast to little endian int16
eend = floor(size(raw,1)/2)*2;
raw16 = typecast(raw(1:eend),'int16');
eend = size(raw16,1);
%raw16 = swapbytes(raw16);

clear raw fileID nrem Index oh ohl
%%
Fs = 16000;
d_raw16 = double(raw16);
sound(d_raw16,Fs);

clear raw16
save(['Audio_' strrep(datestr(datetime('now')),':','_')]);
%%
% Freq analysis

%power spectral density
figure;
[pxx,f] = periodogram(d_raw16(1:eend),hamming(eend),[],Fs);
plot(f,10*log10(pxx))
ylabel('dB') % y-axis label
xlabel('freq [Hz]') % x-axis label
set(gca,'FontSize',20,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',30,'fontWeight','bold')
legend('Raw Data');