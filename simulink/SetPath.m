clear java;

root_path = fullfile(fileparts(mfilename('fullpath')), '../');
addpath(genpath(root_path));
root_path

uiopen('BlueBird_RobotLocation.slx',1)