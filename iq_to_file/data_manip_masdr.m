%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data_manip_masdr.m: by Max Li, 2016
% Reads direct data received from USRP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;

%Read in samples from file
fname = 'Data\usrp_samples_wifi_move_forward_1mph_35-Max_AFI.dat';
fid = fopen(fname,'rb');
tmp = fread(fid,'float');
samples = zeros(length(tmp)/2,2);
samples(:,1) = tmp(1:2:end);
samples(:,2) = tmp(2:2:end);

% Initialize counters
count_sig = 0;
count_match = 0;
count_gps_x = 0;
count_gps_y = 0;
count_gps_z = 0;
count_samp = 0;

% Initialize buffers storing various information saved
match_sig = zeros(nnz(samples(:,2)==1000));
gps_x = zeros(nnz(samples(:,2)==2000));
gps_y = zeros(nnz(samples(:,2)==3000));
gps_z = zeros(nnz(samples(:,2)==4000));
samp_act = zeros(length(tmp)/2 - nnz(samples(:,2)==4000)...
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
                else
                    samp_act(i) = samples(i);
                    count_samp = count_samp + 1;
                end
            end
        end
    end
            
end
mean(samples(:,1))
mean(samp_act(:,1))
%fs = 640000;
