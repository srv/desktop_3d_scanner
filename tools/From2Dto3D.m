function [point3D] = From2Dto3D(point2D,plane,cameraParams)
A =  plane(1);
B =  plane(2);
C =  plane(3);
D =  plane(4);

fx = cameraParams.IntrinsicMatrix(1,1);
fy = cameraParams.IntrinsicMatrix(2,2);
cx = cameraParams.IntrinsicMatrix(3,1);
cy = cameraParams.IntrinsicMatrix(3,2);
u = point2D(1);
v = point2D(2);
rt = [(u-cx)/fx (u-cy)/fy 1];
t = -D/((rt(1)*A)+(rt(2)*B)+C);
point3D = rt * t;
end