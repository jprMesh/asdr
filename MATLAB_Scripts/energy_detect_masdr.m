clear all;
close all;
N = 10000; % N is length of Signal 
Nbins = 1024;
%Set Threshold to check if signal is there.
thresh = 200;

f = 500e3;
Fs = 5e6;
t = (1:N)* 1/Fs;

noise = randn(1,N);
sig = cos(2*pi*f * t);

% Do FFT on time, noise.
fft_noise = fft(noise, Nbins);
fft_sig = fft(sig, Nbins);

mag_noise = abs(fft_noise);
mag_sig = abs(fft_sig);

% Plot FFT results
figure(1);
% Hist values in bins, randn.
plot(-(Nbins-1)/2:(Nbins-1)/2,mag_noise,-(Nbins-1)/2:(Nbins-1)/2,thresh*ones(1,Nbins));
title('FFT Bin Values, Noise');
axis([-512 512 0 250]);

figure(2);
% Hist values in bins, cos.
plot(-(Nbins-1)/2:(Nbins-1)/2,mag_sig,-(Nbins-1)/2:(Nbins-1)/2,thresh*ones(1,Nbins));
axis([-512 512 0 250]);
title('FFT Bin Values, Signal');


% Determine if either value passes threshold.
noise_passes_thresh = (max(mag_noise)>thresh)
sig_passes_thresh = (max(mag_sig)>thresh)