close all
clear
clc

filename = '0588-Cloud';

fileID = fopen([filename,'.txt'],'r');
formatSpec = '%f %f %f %f';
sizeX = [4 Inf];
X = fscanf(fileID,formatSpec,sizeX);
fclose(fileID);

x_ = X(1,:)';
y_ = X(2,:)';
z_ = X(3,:)';
i_ = X(4,:)';

xmin = min(x_); xmax = max(x_);
xmid = (xmin + xmax) / 2;
x_ = x_ - xmid;

ymin = min(y_); ymax = max(y_);
ymid = (ymin + ymax) / 2;
y_ = y_ - ymid;
y_ = -y_;

zmin = min(z_);
z_ = z_ - zmin;

pc = pointCloud([x_,y_,z_],'Intensity',i_);

x_2 = [x_;-x_];
y_2 = [y_;y_];
z_2 = [z_;z_];
i_2 = [i_;i_];

Y = [x_2,y_2,z_2,i_2];
N = size(Y,1);
%Y = Y(:,:);
Y = Y(1:20:N,:);

pc2 = pointCloud([Y(:,1),Y(:,2),Y(:,3)],'Intensity',Y(:,4));

tri = delaunay(Y(:,1),Y(:,2),Y(:,3));
[r,c] = size(tri);

figure;
    subplot(1,2,1);
    pcshow(pc);

    subplot(1,2,2);
    pcshow(pc2);

figure;
    h = trisurf(tri, Y(:,1), Y(:,2), Y(:,3));
    axis equal;

TRI = [tri(:,[1,2,3]); tri(:,[1,2,4]); tri(:,[1,3,4]); tri(:,[2,3,4])];
save('ENVI.mat', 'Y');
save('ENVI.mat', 'TRI', '-append');

%{
filename_output = [filename,'-mirrored.txt'];
fileID = fopen(filename_output,'w');
fprintf(fileID,'%8.3f %8.3f %8.3f %8.2f\n',Y);
fclose(fileID);

unix(['cp ',filename_output,' ~/One*/Public/']);
%}
