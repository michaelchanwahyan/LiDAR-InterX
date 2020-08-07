close all
clear
clc

filename = '0588-Cloud-mirrored-mesh';

fileID = fopen([filename,'.ply'],'r');
formatSpec = '%f %f %f %d';
sizeX = [4 Inf];
X = fscanf(fileID,formatSpec,sizeX);
fclose(fileID);

X = X';

save('ENVI.mat','X');