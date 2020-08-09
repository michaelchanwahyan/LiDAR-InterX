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

roadwidth = 18.5; % meter

% generate LiDAR sensor
    l_orig = [+roadwidth/2 + 5, +roadwidth/2 + 5, 3.5];
    l_tilt = [-pi/7, 0, 3*pi/4];
    l_high = 0.1;
    l_diam = 0.1;

interx;