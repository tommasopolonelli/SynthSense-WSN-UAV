%%
close all
clear all

delay = 0.2;
cycles = 200;
pktsize = 995;
pktover = 3;
% for ranging power consumption
EN_RANGING = 1;

instrreset;

SA = serial('COM3','BaudRate',921600);
SA.InputBufferSize = 2000;
SA.OutputBufferSize = 2000;
ST = serial('COM6','BaudRate',921600);
ST.InputBufferSize = 2000;
ST.OutputBufferSize = 2000;

fopen(SA);
fopen(ST);

fprintf(ST,'S');

%[tlineA,count] = fgetl(SA)
%[tlineT,count] = fgetl(ST)

pause(1)
i = 0;
while EN_RANGING == 1
    %ranging
    pause(0.01);  
    if (SA.BytesAvailable > 0)
        fread(SA,SA.BytesAvailable,'uchar');
        fread(ST,ST.BytesAvailable,'uchar');
        disp('New range');
        disp(i);
        i = i + 1;
    end
    if i > 200
        return
    end
end

fwrite(ST,'E','uchar');
flushinput(ST);
flushinput(SA);
pause(0.5)

%[tlineA,count] = fgetl(SA)
%[tlineT,count] = fgetl(ST)

fwrite(ST,'U','uchar');
pause(0.1)
[~,count] = fread(SA,SA.BytesAvailable,'uchar')

fixsize = [ 0 0 0 0 ];

dealy_v = delay;

pktsize_v = pktsize;
%avoid number multiples of 64
pktsize_v(rem(pktsize_v,64) == 0) = pktsize_v(rem(pktsize_v,64) == 0) + 1;
number_iter = numel(dealy_v) * numel(pktsize_v);
count_iter = 0;
%progress bar
progressBar = waitbar(0,'Generating traffic...','CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
setappdata(progressBar,'canceling',0);

for k=1:numel(pktsize_v)
    pkt = ones(pktsize_v(k),1);
    pkt(1) = 'U';
    for j=1:1:numel(dealy_v)
        tot = 0;
        fail = 0;
        for i=1:cycles
            fwrite(ST,pkt,'uchar');
            pause(dealy_v(j))
            if (SA.BytesAvailable > 0)
                [~,count] = fread(SA,SA.BytesAvailable,'uchar');
                if count <= (pktsize_v(k) + pktover)
                    tot = tot + count;
                else
                    fail = fail + 1;
                end
            else
                pause(0.01)
                fail = fail + 1;
            end
        end
        timetot = cycles * dealy_v(j);
        bps = (tot * 8)/timetot;
        error_rate = fail/cycles;
        fixsize = [fixsize ; pktsize_v(k) dealy_v(j) bps error_rate];
        
        %update progress bar
        if getappdata(progressBar,'canceling')
            delete(progressBar);
            fprintf('\nWarning: terminated by user!\n');
            break
        end
        count_iter = count_iter + 1;
        waitbar(count_iter/number_iter,progressBar,sprintf('Size:%u Del%u bps:%u ER:%u',pktsize_v(k),dealy_v(j),bps,error_rate));
    end
end

delete(progressBar);

%remove first element
fixsize = fixsize(2:end,:);

fclose(SA);
fclose(ST);
