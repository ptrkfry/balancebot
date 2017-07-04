%% compute error
clear all
close all
clc

angle = -89:0.01:89;
stepsErr = 43.5*sin(angle/180*pi)/0.079718;


plot(angle, stepsErr)