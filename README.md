# Sensor Fusion Nanodegree

## Project 4 - Radar Target Generation and Detection

This project implements radar signal generation and CFAR (Constant False Alarm Rate) adaptive thrshold signal processin in Matlab.

---
[img01]: ./img/img01.png " "
[img02]: ./img/img02.png " "


### 1. 2D CFAR

_In a README file, write brief explanations for the following:_

- _Implementation steps for the 2D CFAR process_
- _Selection of Training, Guard cells and offset_
- _Steps taken to suppress the non-thresholded cells at the edges_


#### 1.1 Implementation steps for the 2D CFAR process

The CFAR algorithm does the following steps:

1. Slide a window of size (Tr+Gr+1)x(Td+Gd+1) over the RDM matrix
2. Sum up the signal power levels over the training cells in the window while leaving out the guard cells
3. Convert the average power level back to dB and add the SNR base offset to the threshold
4. For every CUT (= center of the window) assign a value of 1 if the CUT is higher than the threshold, or 0 otherwise

```matlab
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
        
        % Calculate threshold from noise average then add the offset
        threshold = offset + pow2db(noise_level/(2*(Td+Gd+1)*2*(Tr+Gr+1)-(Gr*Gd)-1));
        CUT = RDM(i,j);
        
        if (CUT > threshold)
            RDM(i,j) = max_T;
        else
            RDM(i,j) = 0;
        end
        
    end
end
```

#### 1.2 Selection of Training, Guard cells and offset

The size of the sliding window was set to match the shape of the signal in the surface plot. However, it seems not to be very critcal in this example since a variety of window sizes worked. The size might be more important if the radar picks up more different signals. The SNR offset was estimated with the RDM surf plot.

|Parameters     |       |   |
|---            |---    |---|    
|Training cells |Tr = 16|
|               |Td = 8 |
|Guard cells    |Gr = 8 |
|               |Gd = 4 |
|SNR            |offset = 1.3 dB|


#### 1.3 Steps taken to suppress the non-thresholded cells at the edges

Because of the way the window for the adaptive threshold is slided over the RDM matrix, an unprocessed edge of the half size of the window remains in the result. This edge area was set to 0 to suppress the invalid values.

### 2. RDM and CFAR Results

The following images show the Range Doppler Map of the generated radar signal and the thresholded RDM after applying CFAR (Constand False Alarm Rate) thresholding.

![][img01]

![][img02]
