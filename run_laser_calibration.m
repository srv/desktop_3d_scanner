addpath('../external/epnp');
addpath('../external/ransac');
addpath('../stored_calibrations');
addpath('../laser_calibration');
CameraCalibration=load('C:\Users\Propietario\Documents\GitHub\desktop_3d_scanner\stored_calibrations\cameraCalibration.mat');
cameraParams=CameraCalibration.cameraCalibration;
numImages = 6;
files = cell(1, numImages);
 for i = 1:numImages
    files{i} = fullfile('C:','Program Files','MATLAB', 'TFG', sprintf('bcap%d.jpg', i));
 end
 %% Find the laser plane
 im=[];
 plansC=[];
 plansL=[];
 points3D=[];
 Rp=[];
 Tp=[];
 squareSize = 0.040; %in meters
 A=cameraParams.IntrinsicMatrix(:,1:3);
 kernel = [0.000003	0.000229	0.005977	0.060598	0.24173	0.382925	0.24173	0.060598	0.005977	0.000229	0.000003];
 treshold = 0.8;
 window_size = 7;
for i=1:numImages
%1.-Detect points on image----------------------    
    imOrig = imread(files{i});
    im = undistortImage(imOrig, cameraParams);
    [imagePoints, boardSize] = detectCheckerboardPoints(im);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    figure();
    imshow(im);
    hold on;plot(imagePoints(:,1),imagePoints(:,2),'rx'); 
    hold off;
    [u, v]=detect_laser_subpixel(im, kernel, threshold, window_size,'g');
    laser2D=[u v];
%2.-Inputs format--------------------------------
    x3d_h=[];
    x2d_h=[]; 
    for i=1:length(imagePoints)
        x3d_h(i,:)=[worldPoints(i,:),0,1]; 
        x2d_h(i,:)=[imagePoints(i,:),1];
    end

%3.-EPnP----------------------------------------------------
    [Rp,Tp,Xc,sol]=efficient_pnp(x3d_h,x2d_h,A);
%4.-Find calib plane----------------------------------------
    Rt=Rp';
    n=Rt(:,3);
    D=sum(n.*(Rt*Tp));
    plansC = [plansC; n' D];
%5.-Intersect between point and plane-----------------------
    laser3D=[];
    while(i<=length(laser2D))
      laser3D=[laser3D; From2Dto3D(laser2D(i,:),plansC(i,:),cameraParams)];
    end
    points3D=[points3D; laser3D];
end
laserplane=ransac_fitplane(points3D);
[B,P]=ransacfitplane(points3D,20);


