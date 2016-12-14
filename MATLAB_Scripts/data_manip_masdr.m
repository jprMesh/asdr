%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data_manip_masdr.m: by Max Li, 2016
% Reads direct data received from USRP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;

Ts = 1;
%Ts=42e6;
%Read in samples from file
fname = 'test_wyg_data_12-11.dat';
fid = fopen(fname,'rb');
tmp = fread(fid,'float');
samples = zeros(length(tmp)/2,2);
samples(:,1) = tmp(1:2:end);
samples(:,2) = tmp(2:2:end);
length_temp = length(tmp);
clear tmp;

% Initialize counters
count_sig = 1;
count_match = 1;
count_gps_x = 1;
count_gps_y = 1;
count_gps_z = 1;
count_rss = 1;
count_samp = 1;

% Initialize buffers storing various information saved
match_sig = zeros(nnz(samples(:,2)==1000),1);
gps_x = zeros(nnz(samples(:,2)==2000),1);
gps_y = zeros(nnz(samples(:,2)==3000),1);
gps_z = zeros(nnz(samples(:,2)==4000),1);
rss_val = zeros(nnz(samples(:,2)==5000),1);
samp_act = zeros(length_temp/2 - nnz(samples(:,2)==4000)...
                - nnz(samples(:,2)==3000) - nnz(samples(:,2)==2000)...
                - nnz(samples(:,2)==1000),2);

% Separate Match values, gps values from sampled signals.
for i = 1:length(samples)
    if samples(i,2) == 1000
        match_sig(count_match) = samples(i,1);
        count_match = count_match + 1;
    else if samples(i,2) == 2000
            gps_x(count_gps_x) = samples(i,1);
            count_gps_x = count_gps_x + 1;
        else if samples(i,2) == 3000
                gps_y(count_gps_y) = samples(i,1);
                count_gps_y = count_gps_y + 1;
            else if samples(i,2) == 4000
                    gps_z(count_gps_z) = samples(i,1);
                    count_gps_z = count_gps_z + 1;
                else if samples(i,2) == 5000
                        rss_val(count_rss) = samples(i,1);
                        count_rss = count_rss + 1;
                    else
                        samp_act(i) = samples(i);
                        count_samp = count_samp + 1;
                    end
                end
            end
        end
    end         
end
clear samples;
% mean(samples(:,1));
% mean(samp_act(:,1));
% 
% figure(1);
% plot(1:Ts:(length(samp_act))/Ts,samp_act(:,1));
% title('Sampled Data');

figure(2);
plot(1:Ts:(count_match-1)/Ts,match_sig);
title('Matched Filter Values');

figure(3);
plot(1:Ts:(count_rss-1)/Ts,rss_val);
title('RSS Values');
% 
% %fs = 640000;