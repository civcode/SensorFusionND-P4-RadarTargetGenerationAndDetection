clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
%% User Defined Range and Velocity of target
% define the target's initial position and velocity. Note : Velocity
% remains contant
R0 = 80;  % target range in m
v0 = 25;  % target velocity in m/s
 


%% FMCW Waveform Generation
dr   = 1;       % Range Resolution
Rmax = 200;     % Max. Range
Vmax = 100;      % Max. Velocity
c    = 3e8;     % Speed of Light


%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
B      = c/(2*dr);
Tchirp = 5.5*2.0*Rmax/c;
slope  = B/Tchirp;

% print info
%fprintf("B      = %.2f kHz\n", B/1e6)
%fprintf("Tchirp = %.2f ms\n", Tchirp*1e3)
%fprintf("slope  = %.2f\n", slope)

%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples%

%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    %For each time stamp update the Range of the Target for constant velocity. 
    r = R0 + t(i)*v0;   % range at time t(i)
    
    %For each time sample we need update the transmitted and
    %received signal. 
    dt    = 2*r / c;    % traveling time
    
    Tx(i) = cos(2*pi*(fc*t(i) + (0.5*slope*t(i)^2)));
    Rx(i) = cos(2*pi*(fc*(t(i)-dt) + (0.5*slope*(t(i)-dt)^2)));
    
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i).*Rx(i);  % generate intermediate frequency
    
end

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
MixMat = reshape(Mix, [Nr,Nd]); % Nr ... range cells, Nd ... doppler cells,

%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
MixMat_fft = fft(MixMat);

% Take the absolute value of FFT output
MixMat_fft = abs(MixMat_fft/max(max(MixMat_fft)));

% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
signal_fft = MixMat_fft(1:length(t)/2+1);   % Taking only half of output


%plotting the range
figure ('Name','Range from First FFT')
%subplot(2,1,1)

 % plot FFT output 
 f = length(t) * (0:length(t)/2)/length(t);
 plot(f, signal_fft) 
 axis([0 200 0 1]);
 xlabel('frequency in Hz')
 ylabel('magnitude')
 grid on

 


%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;
RDM = RDM/max(max(RDM));

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);
colorbar;
title('Range Doppler Map')
xlabel('velocity in m/s')
ylabel('distance in m')
zlabel('magnitude')

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

%Select the number of Training Cells in both the dimensions.
Tr = 16;    %10; %12
Td = 8;     %6

%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 8;     %4;
Gd = 4;

% offset the threshold by SNR value in dB
offset = 1.3; %1.4;

%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR
   
max_T = 1;

for i = Tr+Gr+1 : (Nr/2)-(Gr+Tr)
    for j = Td+Gd+1 : Nd-(Gd+Td)
        
        % init noise level
        noise_level = zeros(1,1);
        
        % sum up noise in the area around CUT while
        % leaving out gurad cells
        for p = i-(Tr+Gr) : i+(Tr+Gr)
            for q = j-(Td+Gd) : j+(Td+Gd)
                if (abs(i-p) > Gr || abs(j-q) > Gd)
                    noise_level = noise_level + db2pow(RDM(p,q));
                end
            end
        end
        
        % Calculate threshould from noise average then add the offset
        threshold = offset + pow2db(noise_level/(2*(Td+Gd+1)*2*(Tr+Gr+1)-(Gr*Gd)-1));
        CUT = RDM(i,j);
        
        if (CUT > threshold)
            RDM(i,j) = max_T;
        else
            RDM(i,j) = 0;
        end
        
    end
end

% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
RDM(union(1:(Tr+Gr), end-(Tr+Gr-1):end), :) = 0;  % Rows (velocity)
RDM(:, union(1:(Td+Gd), end-(Td+Gd-1):end)) = 0;  % Columns (distance)


%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
%figure,surf(doppler_axis,range_axis,'replace this with output');
%colorbar;
figure('Name', 'CFAR output')
surf(doppler_axis, range_axis, RDM);
colorbar;
title('CFAR Range Doppler Map')
xlabel('velocity in m/s')
ylabel('distance in m')
zlabel('magnitude')

 
 