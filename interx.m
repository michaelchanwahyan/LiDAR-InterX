close all
clear
clc

% inter-cross
%         |          |
%      (B)|          |(C)
% --------.          .--------
%                    
%                    
%                    
%                    
%                    
% --------.          .--------
%      (A)|          |(D)
%         |          |
%
% (A) := (0,0)

% generate structures
%     Pa              , Pb              , Pc
%     xA    yA    zA    xB    yB    zB    xC    yC    zC
W = [ -100, -100, 0.01, -100,  100, 0.01,  100, -100, 0.01; % ground triangular p1
      -100,  100, 0.01,  100, -100, 0.01,  100,  100, 0.01; % ground triangular p2
      -100, -100, -100, -100,  100, -100, -100, -100,  100; % the Left Wall p1
      -100,  100, -100, -100, -100,  100, -100,  100,  100; % the Left Wall p2
       100, -100, -100,  100,  100, -100,  100, -100,  100; % the Right Wall p1
       100,  100, -100,  100, -100,  100,  100,  100,  100; % the Right Wall p2
      -100, -100, -100, -100, -100,  100,  100, -100, -100; % the Back Wall p1
      -100, -100,  100,  100, -100, -100,  100, -100,  100; % the Back Wall p2
      -100,  100, -100, -100,  100,  100,  100,  100, -100; % the Front Wall p1
      -100,  100,  100,  100,  100, -100,  100,  100,  100; % the Front Wall p2
      -100, -100,  100, -100,  100,  100,  100, -100,  100; % ground triangular p1
      -100,  100,  100,  100, -100,  100,  100,  100,  100; % ground triangular p2
    ];

W = [ W ;
      20, -60, 0, 20, -20,  0, 20, -60, 20;
      20, -20, 0, 20, -60, 20, 20, -20, 20;
      60, -20, 0, 20, -20,  0, 60, -20, 20;
      20, -20, 0, 60, -20, 20, 20, -20, 20;
      
      20,  60, 0, 20,  20,  0, 20,  60, 20;
      20,  20, 0, 20,  60, 20, 20,  20, 20;
      60,  20, 0, 20,  20,  0, 60,  20, 20;
      20,  20, 0, 60,  20, 20, 20,  20, 20;
      
      -20, -60, 0, -20, -20,  0, -20, -60, 20;
      -20, -20, 0, -20, -60, 20, -20, -20, 20;
      -60, -20, 0, -20, -20,  0, -60, -20, 20;
      -20, -20, 0, -60, -20, 20, -20, -20, 20;
      
      -20,  60, 0, -20,  20,  0, -20,  60, 20;
      -20,  20, 0, -20,  60, 20, -20,  20, 20;
      -60,  20, 0, -20,  20,  0, -60,  20, 20;
      -20,  20, 0, -60,  20, 20, -20,  20, 20;
    ];

V = [];
load cardata/private_car_01_data/ENVI.mat % load TRI and Y
Y(:,2) = Y(:,2) - 10;
Y(:,1) = Y(:,1) - 7;
% generate rotation matrix
    vehi_rot = pi/4; % about z axis
    c = cos(vehi_rot); s = sin(vehi_rot); R = eye(4);
    R(1,1) = c; R(1,2) = -s; R(2,1) = s; R(2,2) = c;
% apply rotation to current road user
    Y = Y * R';

V = [ V ;
      [ Y(TRI(:,1),1:3) , Y(TRI(:,2),1:3) , Y(TRI(:,3),1:3) ] ];

load cardata/private_car_01_data/ENVI.mat % load TRI and Y
Y(:,2) = Y(:,2) - 24;
Y(:,1) = Y(:,1) - 7;
V = [ V ;
      [ Y(TRI(:,1),1:3) , Y(TRI(:,2),1:3) , Y(TRI(:,3),1:3) ] ];

load cardata/private_car_01_data/ENVI.mat % load TRI and Y
Y(:,2) = Y(:,2) - 38;
Y(:,1) = Y(:,1) - 7;
V = [ V ;
      [ Y(TRI(:,1),1:3) , Y(TRI(:,2),1:3) , Y(TRI(:,3),1:3) ] ];


WV = [ W ; V ];

wallNum = size(W,1);
vehiNum = size(V,1);

% generate road ground
    roadwidth = 18.5; % meter
    x = -80:1:80;
    y = -80:1:80;
    xx = zeros(numel(x),numel(y));
    yy = zeros(numel(x),numel(y));
    zz = zeros(numel(x),numel(y));
    for i = 1 : numel(x)
        for j = 1 : numel(y)
            xx(i,j) = x(i);
            yy(i,j) = y(j);
            if ((x(i) >= -roadwidth/2 && x(i) <= roadwidth/2) ...
             || (y(j) >= -roadwidth/2 && y(j) <= roadwidth/2))
                zz(i,j) = 0.01;
            end
        end
    end ; clear i j 

% generate LiDAR sensor
    l_orig = [-roadwidth/2 - 5, -roadwidth/2 - 5, 3.5];
    l_tilt = [-pi/7, 0, -pi/4];
    l_high = 0.1;
    l_diam = 0.1;

    l_surfPt = zeros(3600,3);
    for i = 1 : 10
        for j = 1 : 360
            l_surfPt((i-1)*360+j,1) = l_orig(1) + l_diam * cos(j*pi/180);
            l_surfPt((i-1)*360+j,2) = l_orig(1) + l_diam * sin(j*pi/180);
            l_surfPt((i-1)*360+j,3) = l_orig(3) - l_high / 2 + (i-1)*l_high/10;
        end
    end ; clear i j

% generate LiDAR points
l_rps = 5; % rotation per second
l_bm_count = 64;
%l_bm_azimu = 360;
l_bm_azimu = 2048;
l_bm_angElev = linspace(-pi/8, pi/8, l_bm_count);
l_bm_angAzim = linspace(0,2*pi,l_bm_azimu+1);
l_bm_angAzim = l_bm_angAzim(1:l_bm_azimu);

ptNum = l_bm_azimu * l_bm_count;
PCL = zeros(ptNum, 3);

% construct laser beam unit vector u
%     x  y  z
u0 = [0; 1; 0];

ptIdx = 1;
progress_chk_scale = 1000;
progress_intvl = floor(ptNum / progress_chk_scale);
progress_chkpt = 0;
for azim = l_bm_angAzim
    if (ptIdx > progress_chkpt)
        progress_chkpt = progress_chkpt + progress_intvl;
        t = datetime('now');
        fprintf('[ %s ] rendering progress = %d / %d\n', t,  ptIdx, ptNum);
    end
    for elev = l_bm_angElev
        c = cos(elev); s = sin(elev);
        R = [ 1 , 0 , 0 ;   % for elevation turning
              0 , c ,-s ;   % which is a rotation about the x-axis of u0
              0 , s , c ];
        u = R * u0;         % perform elevation turning
        c = cos(-azim); s = sin(-azim);
        R = [ c ,-s , 0 ;   % for azimuth turning
              s , c , 0 ;   % which is a rotation about the z-axis of current u
              0 , 0 , 1 ];
        u = R * u;          % perform azimuth turning
        c = cos(l_tilt(1)); s = sin(l_tilt(1));
        R = [ 1 , 0 , 0 ;   % for tilting of LiDAR elevation
              0 , c ,-s ;   % which is a rotation about x axis
              0 , s , c ];
        u = R * u;
        c = cos(l_tilt(3)); s = sin(l_tilt(3));
        R = [ c ,-s , 0 ;   % for tilting of LiDAR elevation
              s , c , 0 ;   % which is a rotation about x axis
              0 , 0 , 1 ];
        u = R * u;
        
        % along the laser beam unit vector, emit a pulse and hit the walls
        surfCandi = false(1,wallNum + vehiNum); % surface candidates wrt WL
        gamma_arr = zeros(1,wallNum + vehiNum); % gamma value candidates
        for wvIdx = 1 : wallNum + vehiNum
            [ surfCandi(wvIdx) , gamma_arr(wvIdx) ] = checkHit( WV(wvIdx,1:3) , WV(wvIdx,4:6) , WV(wvIdx,7:9), u' , l_orig );
        end ; clear wIdx
        gamma_ = min( gamma_arr( surfCandi ) );
        
        % save the point
        PCL(ptIdx,:) = l_orig + gamma_ * u';
        ptIdx = ptIdx + 1;
    end
end ; clear azi ela

PCL = PCL( PCL(:,1) >= -80 & PCL(:,1) <= 80 ...
    & PCL(:,2) >= -80 & PCL(:,2) <= 80 ...
    & PCL(:,3) <=6 , : );

log_fname = 'pc-'+ string(datetime('now', 'Format', 'y-m-dd-HH-mm-ss'))+ '.mat';
save(log_fname, 'PCL');
save(log_fname, 'log_fname', '-append');

% plotting
figure;
h1 = surf(xx,yy,zz);
    set(h1, 'EdgeColor', 'none');
    axis([-80,80,-80,80,-5,80]); % axis equal
    hold on;

h2 = plot3(l_surfPt(:,1), l_surfPt(:,2), l_surfPt(:,3), 'k.');
    set(h2, 'MarkerSize', 0.1);
    axis equal;
    hold on;

h3 = plot3(0,0,30,'k.');
	axis equal;
    hold on;

h4 = plot3(PCL(:,1), PCL(:,2), PCL(:,3), 'r*');
    set(h4,'MarkerSize', 0.5);
    axis equal;
    hold on;

pc = pointCloud(PCL);
figure;
    pcshow(pc);
    colormap hsv;
% combo of making gif from videos:
% ffmpeg -i video.mp4 frame%04d.png
% +
% gifski -o file.gif frame*.png
