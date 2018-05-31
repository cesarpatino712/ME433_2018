function plotFFT( x )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if mod(length(x),2) == 1   %x should have even number of samples
    x = [x 0];             %padding
end
N  =length(x);
X = fft(x);
mag(1) = abs(X(1))/N;           %DC component
mag(N/2+1) = abs(X(N/2+1))/N;   %Nyquist frequency component
mag(2:N/2) = 2*abs(X(2:N/2))/N; %all other frequency components
freqs  =linspace(0,1,N/2+1);    %make x-axis as fraction of Nyquist freq
stem(freqs, mag);               %plot FFT magnitude plot
axis([-0.5 1.05 -0.1*max(mag) 1.1*max(mag)]);
xlabel('Frequency (as fraction of Nyquist frequency)');
ylabel('Magnitude');
title('Single-Sided FFT Magnitude');
set(gca,'FontSize',18);




