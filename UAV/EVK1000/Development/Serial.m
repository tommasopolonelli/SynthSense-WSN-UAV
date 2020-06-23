%%
close all
clear all

delay = 0.02;
cycles = 200;
pktsize = 995;
pktover = 3;
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

%optimal for 110kbps
%dealy_v = delay:delay:0.2;
%optimal for 6.8Mbps
dealy_v = linspace(0.001,0.02,10);

pktsize_v = ceil(linspace(10,pktsize,10));
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


% pkt = ones(pktsize,1);
% pkt(1) = 'U';
% pkt = uint8(pkt);
% for j=1:1:numel(dealy_v)
%     tot = 0;
%     fail = 0;
%     for i=1:cycles
%         fwrite(ST,pkt,'uchar')
%         pause(dealy_v(j))
%         if (SA.BytesAvailable > 0)
%             [~,count] = fread(SA,SA.BytesAvailable,'uchar')
%             if count <= (pktsize_v(k) + pktover)
%                 tot = tot + count;
%             else
%                 fail = fail + 1;
%             end
%         else
%             pause(0.01)
%             fail = fail + 1;
%         end
%     end
%     timetot = cycles * dealy_v(j);
%     bps = (tot * 8)/timetot;
%     error_rate = fail/cycles;
%     fixsize = [fixsize ; pktsize_v(k) dealy_v(j) bps error_rate];
%     
%     %update progress bar
%     if getappdata(progressBar,'canceling')
%         delete(progressBar);
%         fprintf('\nWarning: terminated by user!\n');
%         break
%     end
%     count_iter = count_iter + 1;
%     waitbar(count_iter/numel(dealy_v),progressBar,sprintf('Pkt_size:%u Delay:%u bps:%u ER:%u',pktsize,dealy_v(j),bps,error_rate));
% end



delete(progressBar);

%remove first element
fixsize = fixsize(2:end,:);

fclose(SA);
fclose(ST);

%%

bitrate = reshape(fixsize(:,3),[numel(pktsize_v) numel(dealy_v)]);
percerror = reshape(fixsize(:,4),[numel(pktsize_v) numel(dealy_v)]);
%tradeof between bitrate and packet loss */
tr = percerror ./ bitrate;

figure;
surf(pktsize_v,dealy_v,floor(bitrate),percerror.*100,'FaceColor','interp');
colorbar
title('Throughput and PER High-Speed')
zlabel('bps') % x-axis label
ylabel('Packet period [s]') % x-axis label
xlabel('Packet Size [B]') % x-axis label
set(gca,'FontSize',24,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',24,'fontWeight','bold')
grid on
hold on
idx = tr < mean(mean(tr));
pktsize_a = repmat(pktsize_v,numel(dealy_v));
dealy_a = repmat(dealy_v,numel(dealy_v))';
plot3(pktsize_a(idx),dealy_a(idx),floor(bitrate(idx)),'.r','markersize',20);
%imagesc(pktsize_v,dealy_v,percerror.*100);
hold off

