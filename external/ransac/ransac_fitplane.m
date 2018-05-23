function [M]=ransac_fitplane(d)
s=3;  %minimo numero de datos requeridos para el modelo
n=10; %numero iteraciones
U=1; %umbral para saber si un dato encaja
T=1080;  %numero de puntos que puede soportar el modelo 
i=0;
inliers=[];
M=[];
E=inf;
while(i<n) %Hacer N veces
    %Elegimos s datos de D de manera aleatoria
    ds1=d(randi(1080),:);
    ds2=d(randi(1080),:);
    ds3=d(randi(1080),:);
    %Encontramos un modelo M que encaje en esos datos
    u=ds2-ds1;
    v=ds3-ds1;
    A=det([u(2) v(2);u(3) v(3)]);
    B=-det([u(1) v(1);u(3) v(3)]);
    C=det([u(1) v(1);u(2) v(2)]);
    D=-A*ds1(1)-B*ds1(2)-C*ds1(3);
    m=[A B C D];
    %Para cada dato fuera de s, comprobar si la distacia a M está por
    %debajo de U
    dist= abs(d(:,1)*A+d(:,2)*B+d(:,3)*C+D)/sqrt(A*A+B*B+C*C);
    indx=find(dist<U);
    inliers=d(indx,:);
    %Calcular el error cometido
    e=0;
    %Si num inliers es superior a T y el error es mejor, guardamos M y E
    if(length(inliers)>=T && e<=E)
        M=m;
        E=e;
    end
    i=i+1;
end
end
