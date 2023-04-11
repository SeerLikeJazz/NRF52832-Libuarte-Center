%作者：念通智能
%采样率：500Hz
%数据格式：包头 + EEG数据 +校验 + 包序号
%BBAA + 24x9 + 校验位 + 包序号 = 220字节
%EEG：存原始数据
%EMG_Sequence：包序号

clear
clc

delete(instrfindall);
scom = 'COM9'; %name of serial
Baudrate = 921600; %serial baudrate, should be same as the mcu setting
b = serial(scom);
b.InputBufferSize=2500;

set(b,'BaudRate',Baudrate);
fopen(b);
pause(1)
%%%%%%%%%%%%%%修改打开串口的方式_END %%%%%%%%%%%%%%%%%
EMG_CHANNEL=8;
emg_cnt_max=EMG_CHANNEL*3*9;

EMG_bytes=zeros(1,emg_cnt_max);%%%记录一帧的字节数据
emg_cnt_state=0;%%%记录switch函数的状态
emg_cnt_count=0;%%%每次计数到emg_cnt_max为止
emg_idx=1;

EMG_frame=zeros(EMG_CHANNEL,9);%%%数据解析后的数据帧

result_emg=zeros(EMG_CHANNEL,2502);%%%绘图更新缓冲区
result_emg_idx=1;


fig=figure();
hold on;
for k=1:8
    subplot(4,2,k);
    line_EMG{k}=plot((1:size(result_emg,2))/500,result_emg(k,:));

%     ylim([-3300000/2/2500,3300000/2/2500])
    ylabel('幅值(uV)');
    xlim([0,size(result_emg,2)/500])
    xlabel('时间(s)');
end


drawnow();
% 清空缓冲区%
% for i=1:10
%     fread(b,1000,'uint8');
% end


while true
    [buff,count]=fread(b,1000,'uint8');
    for index=1:length(buff)
        switch(emg_cnt_state)
            case 0
                if(buff(index)==187) %0xBB
                    emg_cnt_state=1;
                else 
                    emg_cnt_state=0;
                end
            case 1
                if(buff(index)==170) %0xAA
                    emg_cnt_state=2;
                    emg_cnt_count=1;
                    emg_sumchkm = 0;
                elseif(buff(index)==187)%排除0xBB BB AA的情况
                    emg_cnt_state=1;
                else
                    emg_cnt_state=0;
                end               
            case 2
                EMG_bytes(1,emg_cnt_count) = buff(index);
                emg_cnt_count = emg_cnt_count + 1;
                emg_sumchkm = emg_sumchkm + buff(index);
                if(emg_cnt_count == emg_cnt_max+1)%%判断数据接收是否结束
                    emg_cnt_state=3;
                else
                    emg_cnt_state=2;
                end    
            case 3
                if(buff(index) == 0)%%检查校验位
                    emg_cnt_state=4;
                else
                    emg_cnt_state=0;
                end
            case 4
                EMG_Sequence(emg_idx,1) = buff(index);%%%记录包序号

                % 原始数据解析%

                for i=1:9
                    for j=1:8
                       if(EMG_bytes(1,j*3-2+24*(i-1)) > 127)%%%第8位是否为1
                           EMG_frame(j,i) = swapbytes(typecast(uint8([255 EMG_bytes((j*3-2+24*(i-1)):j*3+24*(i-1))]),'int32'));
                       else
                           EMG_frame(j,i) = swapbytes(typecast(uint8([0   EMG_bytes((j*3-2+24*(i-1)):j*3+24*(i-1))]),'int32'));                 
                       end                         
                    end
                end
                EMG(1:8,(emg_idx-1)*9+1:emg_idx*9) = EMG_frame;
                emg_idx = emg_idx + 1;
                emg_cnt_state=0;  % switch状态切换%

    
                % 绘图缓存填充%
                result_emg(:,(result_emg_idx-1)*9+1:result_emg_idx*9)=EMG_frame*0.02235174;
%                 if result_emg_idx == 278  %%%278*9=2502
%                     result_emg=zeros(EMG_CHANNEL,2502);
%                 end
                result_emg_idx=mod(result_emg_idx,278)+1;
               
        end
    end
    %%刷新图像
    for k=1:8
        set(line_EMG{k},'YData',result_emg(k,:));
    end
    drawnow();   

end


AA = diff(EMG_Sequence);
BB=find(AA~=1);
CC=diff(BB);