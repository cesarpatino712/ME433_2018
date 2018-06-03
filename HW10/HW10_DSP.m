 
x = 0;
x(1:10) = 2;
x(11:20) = 0;
x = [x x x x x x x x x x x x 2*ones(1,10) zeros(1,6)];
N = 2^nextpow2(length(x));      %compute number of zeros to pad
xpad = [x zeros(1,N-length(x))];    %add the zero padding
%PLOT
%figure(1)
%plot(x, 'bo');
%axis([-5 205 -0.1 2.2]);
%ylabel('x(k)'); xlabel('Sample number')
%title('Single-sided FFT magnitude');
%set(gca, 'FontSize', 18);
%FFT
%figure(2)
%plotFFT(x);

%inverse FFT
NN  =length(x);
X = fft(x);
xrecovered = fft(conj(X)/N);
%figure(3)
%plot(real(xrecovered));

%Cutoff Freq = 5Hz
%Sampling rate = 100 Hz, Nyquist freq = 1/2 Sampling = 50 Hz: cutoff = 0.1
%Nyquist
b = fir1(7,0.1);               %10th order, 11 sample LPF with cutoff freq of 0.2 fN
freqz(b)

%receive data from the PIC
if ~isempty(instrfind) %closes the port if it was open
    fclose(instrfind);
    delete(instrfind);
end

ser = serial('COM3','BaudRate', 2304000);
fopen(ser);
fprintf(ser,'r');
data = zeros(100, 4);
for i = 1:100
    data(i,:) = fscanf(ser, '%f %f %f %f');
end

xx = linspace(1,100);

plot(xx,data(:,1),xx,data(:,2),xx,data(:,3),xx,data(:,4))
legend('original','MAF','FIR','IIR')

