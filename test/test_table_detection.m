I1 = rgb2gray(imread('https://es.mathworks.com/help/examples/vision/win64/FeatureBasedObjectDetectionExample_01.png'));
I2 = rgb2gray(imread('https://es.mathworks.com/help/examples/vision/win64/FeatureBasedObjectDetectionExample_02.png'));
%Markers and examples images
m1=rgb2gray(imread('Marker1.jpg'));
m2=rgb2gray(imread('Marker2.jpg'));
m3=rgb2gray(imread('Marker3.jpg'));
e1=rgb2gray(imread('example1.jpg'));
e2=rgb2gray(imread('example2.jpg'));
e3=rgb2gray(imread('example3.jpg'));
e4=rgb2gray(imread('example4.jpg'));

% Remove title and borders
I1 = m2;             %I1(100:(end-120),150:(end-150));
I2 = e4;            %I2(100:(end-120),150:(end-150));

points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);

[f1, vpts1] = extractFeatures(I1, points1);
[f2, vpts2] = extractFeatures(I2, points2);

% Set the 'Unique' parameter to true to remove non-unique matches
indexPairs = matchFeatures(f1, f2,...
  'Method', 'Exhaustive',...
  'MatchThreshold', 4.0,...
  'Unique', false,...                    
  'MaxRatio', 0.3);

matchedPoints1 = vpts1(indexPairs(:, 1));
matchedPoints2 = vpts2(indexPairs(:, 2));

[tform, inlier1, inlier2, status] = estimateGeometricTransform(...
  matchedPoints1, matchedPoints2, 'projective',...
  'MaxDistance', 1.2);

boxPolygon = [1, 1;...                           % top-left
        size(I1, 2), 1;...                 % top-right
        size(I1, 2), size(I1, 1);... % bottom-right
        1, size(I1, 1);...                 % bottom-left
        1, 1];                   % top-left again to close the polygon
newBoxPolygon = transformPointsForward(tform, boxPolygon);

figure()
showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2);
legend('unique matched points 1','unique matched points 2');
figure()
imshow(I2); hold on
line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'y');