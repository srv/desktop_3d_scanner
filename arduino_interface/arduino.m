s=serial('COM6','BAUD', 9600); % Make sure the baud rate and COM port is 
                              % same as in Arduino IDE
fopen(s);
pause(1);

for m=1:2 
    servalue=m
    fprintf(s,servalue);          %This command will send entered value to Arduino 
    pause(1);
    fprintf(s,0); 
    pause(1);
    a = fscanf(s,'%d')
end
fclose(s);
delete(s);
clear s;