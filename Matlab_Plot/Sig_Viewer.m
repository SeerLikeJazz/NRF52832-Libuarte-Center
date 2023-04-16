%作者：念通智能
%采样率：500Hz
%数据格式：包头 + EEG数据 +校验 + 包序号
%BBAA + 24x9 + 校验位 + 包序号 = 220字节
%EEG：存原始数据
%EMG_Sequence：包序号
%校验位还未检查

clear
clc

delete(instrfindall);
scom = 'COM4'; %name of serial
Baudrate = 921600; %serial baudrate, should be same as the mcu setting
b = serial(scom);
b.InputBufferSize=25000;

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
EMG_frame=zeros(9,EMG_CHANNEL);%%%数据解析后的数据帧

result_emg=zeros(2502,EMG_CHANNEL);%%%绘图更新缓冲区
result_emg_idx=1;

%设置图的数据源
T=['result_emg(:,  )'];
for Z=1:8
    line_EMG{Z}=plot(result_emg(:,Z));
    if Z<10
        T(end-1)=string(Z);
    else
        T(end-2:end-1)=string(Z);
    end
    set(line_EMG{Z},'YDataSource',T);  % 设置y轴数据来源 data1
    hold on      % 为了让多条图线在一个图中显示
end
%设置垂直线
XL = xline(result_emg_idx*9);

%发送指令
fwrite(b,'T');

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
                    disp(emg_idx)%%校验错误,打印当前帧位置和数据
                end
            case 4
                EMG_Sequence(emg_idx,1) = buff(index);%%%记录包序号

                % 原始数据解析%

                for i=1:9
                    for j=1:8
                       if(EMG_bytes(1,j*3-2+24*(i-1)) > 127)%%%第8位是否为1
                           EMG_frame(i,j) = swapbytes(typecast(uint8([255 EMG_bytes((j*3-2+24*(i-1)):j*3+24*(i-1))]),'int32'))+8000*j;
                       else
                           EMG_frame(i,j) = swapbytes(typecast(uint8([0   EMG_bytes((j*3-2+24*(i-1)):j*3+24*(i-1))]),'int32'))+8000*j;                 
                       end                         
                    end
                end
                emg_idx = emg_idx + 1;
                emg_cnt_state=0;  % switch状态切换%
                % 绘图缓存填充%
                result_emg((result_emg_idx-1)*9+1:result_emg_idx*9,:)=EMG_frame*0.02235174;
                result_emg_idx=mod(result_emg_idx,278)+1;
               
        end
    end
    %%刷新图像
    refreshdata
    XL.Value = result_emg_idx*9;   
    drawnow  
end


AA = diff(EMG_Sequence);
BB=find((AA~=1)&(AA~=-255));