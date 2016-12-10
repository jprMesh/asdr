clear all;
close all;

%Seed to generate expected value (so threshold is always applicable).
rng(500);
%Size of random signal, representing noise
N=400;
%pick thresh to separate matched result to unmatched result.
thresh=20;

%Example packet to look for
toMatch = [1 -1 -1 -1 1 1 -1 1 1 -1 -1 -1 -1 -1 1 1 1 -1 1 1 1 1 -1 -1 1];
%Reverse, so a filter operation will instead correlate. Shifted back to
%positive to be causal.
reverseMatch = toMatch(end:-1:1);

%Generate random signal.
noise = randn(1,N);
%make signal with noise and 
noise_signal = noise;

%Add expected packet to signal.
for i=50:length(toMatch) + 49
    noise_signal(i) =  noise(i)+toMatch(i-49);
end

figure(1);
plot(noise_signal);
title('Noisy Signal With Match Header');

%filter noise_signal
filt_sig= filter(reverseMatch,1, noise_signal);
figure(2);
plot(1:length(filt_sig),filt_sig,1:length(filt_sig),thresh*ones(length(filt_sig)));
title('Match Filtered Signal');