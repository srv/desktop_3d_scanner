function [u, v] = detect_laser_subpixel(rgb_image, kernel, threshold, window_size,laserColor)
% DETECT_LASER_SUBPIXEL Detects laser in an RGB image
%
%  [u, v] = DETECT_LASER_SUBPIXEL(rgb_image, kernel, threshold, window_size,laserColor)
%
%  Common parameters:
%   kernel = [0.000003	0.000229	0.005977	0.060598	0.24173	0.382925	0.24173	0.060598	0.005977	0.000229	0.000003];
%   threshold = 0.8
%   window_size = 7
%   laserColor= 'g' (green)

if (laserColor=='r')
    laserColor=1;
elseif (laserColor=='g')
    laserColor=2;
end

red_ch = double(rgb_image(:,:,laserColor)) / 255.0;
%Create the convolution
kernel_size = size(kernel, 2);
c = conv2(red_ch,kernel);
[~, indices] = max(c, [], 2);
peaks = [];
for i=1:size(rgb_image,1)
  j =  indices(i)-(kernel_size-1)/2;
  % Detect only high values for laser
  if red_ch(i, j) > threshold
    plot(j, i, 'rx')
    peaks = [peaks; i j];
  end
end

% Get subpixel accuracy: Compute the Gaussian approximation or weight mass
% centre of the surrounding pixels for each detected peak:
new_peaks = [];
for i=1:size(peaks)
 col = peaks(i, 2);
 row = peaks(i, 1);
 val = log(red_ch(i, col));
 valAnt = log(red_ch(i, col-1)); 
 valAfter = log(red_ch(i, col+1));
 offset= 1/2*((valAnt-valAfter)/(valAnt+valAfter-2*val));
 if(abs(offset)>1)
    % Get subpixel accuracy mass centre
    % Compute the weight mass centre of the surrounding pixels
    window_size = 3; 
    mx = 0;
    m = 0;
  for k=-(window_size-1)/2:(window_size+1)/2
    if (col + k > size(red_ch,2))
      continue
    end
    if (col + k < 0)
      continue
    end
    value = red_ch(row, col+k);
    mx = mx + value*k;
    m = m + value;
  end
  offset = mx / m;
 end
 new_peaks = [new_peaks; row col+offset];
end


u = new_peaks(:,1);
v = new_peaks(:,2);