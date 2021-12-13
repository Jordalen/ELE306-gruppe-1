clc
clear

path = [12.00   12.00   0;
        22.00   12.00   0;
        27.00   17.00   pi/2;
        27.00   57.00   pi/2;
        27.00   12.00   pi/2;
        32.00   7.00    0;
        82.00   7.00   0]; 
n=0;
rate=robotics.Rate(1);
while n<10
    n = n+1;
    x0 = path(n,:);
    xg = path(n+1,:);
    waitfor(rate);
    
 
end
sl_drivepose

r = sim('sl_drivepose')