addpath('../tools');

I = imread('test_image.jpg');
kernel = [0.000003	0.000229	0.005977	0.060598	0.24173	0.382925	0.24173	0.060598	0.005977	0.000229	0.000003];
treshold = 0.8;
window_size = 7;

[u, v] = detect_laser_subpixel(I, kernel, threshold, window_size);

close all;
figure(1)
imshow(I);
hold on;
plot(u, v, 'go','MarkerSize', 20);
