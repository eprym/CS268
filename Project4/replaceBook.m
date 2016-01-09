function replaceBook()
% Replace the book cover with a new book
%   img1 : the original book cover
%   img2 : the book cover we want to replace with
%   maxIter : the max number of iteration in ransac
%   threshold : the inlier threshold in ransac

close all;
clear;
clc;
load('G.mat');
datapath1 = 'Data/images/';
datapath2 = 'Data/';
datapath3 = 'Data/depths/';

Depths    = dir([datapath3, '*.mat']);
path = [datapath2 'orange.png'];
orig_book = double(imresize(imread(path),0.5));
path = [datapath2 'blue1.png'];
replace_book = double(imresize(imread(path),0.5));

Istart = 1;
numI = 18;
framePeriod = 1;

depthPath = [datapath3 Depths(Istart).name];
temp      = load(depthPath);
DIm1      = temp.depth;

path = sprintf([datapath1 '%03d.png'], Istart);
I1   = double(imresize(imread(path), 1));

[P C] = insertLambertianPlane(I1, orig_book, replace_book, DIm1);

G_last = G{1};
P = G{1}*[P; ones(1,size(P,2))]; %Transform the points into the common reference frame     
P(4,:) = [];
I_last = I1; DIm_last = DIm1;

for i = Istart+framePeriod : framePeriod : numI*framePeriod+Istart
    path = sprintf([datapath1 '%03d.png'], i);
    Ii   = double(imresize(imread(path), 1)); % the i-th image
        
    depthPath = [datapath3 Depths(i).name];
    temp      = load(depthPath);
    DImi      = temp.depth;
    if i<6
        [Pi Ci] = insertLambertianPlane(Ii, orig_book, replace_book, DImi);
    elseif i == 6 || i == 7
        G_last = G_last * G{i};
        continue;
    else
        [Pi Ci] = getPointcloud(Ii, DImi);
    end
    
    Pi = G_last * G{i}*[Pi; ones(1,size(Pi,2))];
    Pi(4,:) = [];
    G_last = G_last * G{i};
    P = [P Pi];
    C = [C Ci];
end
  
P(2,:) = -P(2,:);
ptCloud = pointCloud(P','Color', uint8(C'));
pcwrite(ptCloud, 'ptcloud_newbook');

% I  = GenerateImageFromPoints(P,C);
% figure;
% imagesc(I/255); drawnow;
end

