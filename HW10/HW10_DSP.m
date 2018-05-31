%HW10_DSP Digital Signal Proessing
x = 0;
x(1:10) = 2;
x(11:20) = 0;
x = [x x x x x x x x x x x x 2*ones(1,10) zeros(1,6)];
N = 2^nextpow2(length(x));      %compute number of zeros to pad
xpad = [x zeros(1,N-length(x))];    %add the zero padding
%PLOT
figure(1)
plot(x, 'bo');
axis([-5 205 -0.1 2.2]);
ylabel('x(k)'); xlabel('Sample number')
title('Single-sided FFT magnitude');
set(gca, 'FontSize', 18);
%FFT
figure(2)
plotFFT(x);

%inverse FFT
NN  =length(x);
X = fft(x);
xrecovered = fft(conj(X)/N);
figure(3)
plot(real(xrecovered));

b = fir1(10,0.2);               %10th order, 11 sample LPF with cutoff freq of 0.2 fN
freqz(b)


