%CS268 HW4 Sample Code Test Script
%Load vlfeat and vlg if necessary
%% %%%%%%%%%%%%%%%%% LOADING THE FIRST IMAGE AND PREPARING SOME PLOTS %%%%%%%%%%%%%%%%%
clear all
close all
clc

%Prep some paths relative to this file
datapath1 = 'Data/images/';
datapath2 = 'Data/';
datapath3 = 'Data/depths/';

% Focal Lengths: 533.15 534.51 (fuRgb, fvRgb)
% Principal Point: 315.30 245.98 (cuRgb, cvRgbEdit)
% Radial (OpenCV Style): 0.0728 -0.3772  0.4652 (k0, k1, k4)
% (tangential) 0.0060 -0.0002 (k2, k3)

%Get the depth files
Depths    = dir([datapath3, '*.mat']);
%Do we want to do anything with the images of the books? Load them!
Iy = imresize(imread([datapath2 'yellow.png']), 1); % There's another version of this file in the data folder as well
Ib = imresize(imread([datapath2 'blue2.png']), 1);
% Note! If you want to replace the books that appear in the scene you
% should try using these images

%Load the first image and point cloud
Istart      = 1; %starting from
i=Istart;
%Depth image loading
depthPath = [datapath3 Depths(i).name];
temp      = load(depthPath);
DIm1      = temp.depth;
%color image loading
path = sprintf([datapath1 '%03d.png'], i);
I1   = double(imresize(imread(path), 1));

%Let's visualize what we've loaded so far
figure(2); clf;
subplot(1,2,1); 
imagesc(I1/255); axis image; title('Color Image');
subplot(1,2,2); 
imagesc(DIm1); axis image; title('Depth Image');

%Build the pointcloud corresponding to this first image and depth map
[P1 C1] = getPointcloud(I1,DIm1);
%Let's visualize this pointcloud as a sanity check
%figure(4); clf;
%subsample the cloud so that it doesn't kill the matlab visualization tool
% samples = randperm(min(size(P,2)));
% if(numel(samples) > 0)
%     samples = samples(1:min(numel(samples), 500)); %If your computer can't handle the 3D visualization, reduce this number
% 
%     %Go and plot the point cloud so far
%     subplot(1,2,1);
%     scatter3(P(1,samples),P(2,samples),P(3,samples),45,mean(C(:,samples),1), '.', 'LineWidth', 0.0001)
%     colormap(gray(256)); hold on; xlabel('x'); ylabel('y'); zlabel('z');
%     view([0 -90]); ylim([-0.6,0.6]); axis manual;
%     subplot(1,2,2);
%     scatter3(P(1,samples),P(2,samples),P(3,samples),45,mean(C(:,samples),1), '.', 'LineWidth', 0.0001)
%     colormap(gray(256)); hold on; xlabel('x'); ylabel('y'); zlabel('z');
%     view([0 0]); zlim([0.3,2.2]); axis manual;
% end
%% %%%%%%%%%%%%%%%%% ALIGNING THE POINTCLOUDS FROM THE IMAGES WE WANT INTO A COMMON COORDINATE FRAME %%%%%%%%%%%%%%%%%
%Now let's go through all the rest of the pointclouds and align them to the
%first one we just loaded. The choice of reference frame is arbitrary, so
%you can choose something other than the camera frame of image 1 if you
%would like.

%Which set of frame do we want to align

framePeriod = 1; %sample with period
numI        = 18; %this many frames

% G{Istart} = eye(4); %homogeneous form of transformation
% R{Istart} = eye(3); %List of rotations and translations
% T{Istart} = [0;0;0];

P = P1; C = C1;
I_last = I1; DIm_last = DIm1; G_last = eye(4);G{1} = G_last;
for i=Istart+framePeriod:framePeriod:numI*framePeriod+Istart
    %Let's load the frame we want to align
    fprintf('processing image %02d ...\n', i);
    path = sprintf([datapath1 '%03d.png'], i);
    Ii   = double(imresize(imread(path), 1));

    %Load the depth maps we want
    depthPath = [datapath3 Depths(i).name];
    temp      = load(depthPath);
    DImi      = temp.depth;
    
    %Get the pointcloud and colors out
    [Pi Ci] = getPointcloud(Ii,DImi);

    % CS268 HW4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Get the R and T that you need to put the point cloud into the right
    %coordinate frame
    %[R{i}, T{i}] = ???????????;
    %G{i}         = ???????????;
    
    [inliers1, inliers2] = extractInliers(I_last, Ii);
    DIm_last_match = zeros(size(DIm_last));
    DImi_match = zeros(size(DImi));
    for j = 1:size(inliers1,2)
        DIm_last_match(inliers1(2,j), inliers1(1,j)) = DIm_last(inliers1(2,j), inliers1(1,j));
        DImi_match(inliers2(2,j), inliers2(1,j)) = DImi(inliers2(2,j), inliers2(1,j));
    end
    [P_last_inlier, C_last_inlier] = getPointcloud(I_last, DIm_last_match);
    [P_current_inlier, C_current_inlier] = getPointcloud(Ii, DImi_match);
%     [P_last_inlier, C_last_inlier] = getPointCloudInlier(I_last, DIm_last, inliers1);
%     [P_current_inlier, C_current_inlier] = getPointCloudInlier(Ii, DImi, inliers2);
    
    %showPointCloud(P_last_inlier',C_last_inlier'/255 );
     [R{i}, T{i}, E{i}] = icp(P_last_inlier, P_current_inlier);
%     R{i}
%     T{i}
     G{i} = [R{i} T{i}; 0 0 0 1];
%     
    %G{i} = icpMex(P_last_inlier,P_current_inlier,eye(4),-1,'point_to_point');
%     pt_last = pointCloud(P_last_inlier');
%     pt_current = pointCloud(P_current_inlier');
%     tform{i} = pcregrigid(pt_current, pt_last);
%     G{i} = tform{i}.T;
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Now use R and T to align the cloud to our first image
    Pi = G_last * G{i}*[Pi; ones(1,size(Pi,2))]; %Transform the points into the common reference frame     
    Pi(4,:) = [];
    G_last = G_last * G{i};
    I_last = Ii; DIm_last = DImi;
    %Add the transformed points to our main pointcloud
    P = [P Pi];
    C = [C Ci];
    
    %Add this to the visualization 
%     figure(4);
%     showPointCloud(P',C'/255 );
    %Some more subsampling
%     samples = randperm(min(size(Pi,2),size(Pi,2)));
%     if(numel(samples) > 1)
%         samples = samples(1:500); %If your computer can't handle the 3D visualization, reduce this number
% 
%         %Add the sampled new points to our sanity check visualization
%         subplot(1,2,1)
%         scatter3(Pi(1,samples),Pi(2,samples),Pi(3,samples),45,mean(Ci(:,samples),1), '.', 'LineWidth', 0.0001)
%         colormap(gray(256)); hold on; xlabel('x'); ylabel('y'); zlabel('z');
%         view([0 -90]); ylim([-0.6,0.6]); axis manual;
%         subplot(1,2,2);
%         scatter3(Pi(1,samples)',Pi(2,samples)',Pi(3,samples)',45,mean(Ci(:,samples),1), '.', 'LineWidth', 0.0001)
%         colormap(gray(256));
%         hold on; xlabel('x'); ylabel('y'); zlabel('z'); view([0 0]); zlim([0.3,2.2]); axis manual;
%     end
end
% figure;

savedP = P;
savedC = C;

%% %%%%%%%%%%%%%%%%% THE FUN PART! LET'S PLAY WITH THE SCENE WE HAVE RECONSTRUCTED AND RENDER NEW IMAGES OF IT %%%%%%%%%%%%%%%%%
%Let's do fun and interesting things with the pointcloud now!

% CS268 HW4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Insert some random things! Here are a couple examples of things you could
%do. You can use the image of the yellow book to do interesting things with
%the planes you know how to find in the images, you can add random objects
%(possibly animated!), or do whatever you fancy. 
%[outP1,outC1] = insertLambertianSphere(?);
%[outP2,outC2] = insertLambertianPlane(?);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%dummy code
addedP = [];
addedC = [];

%Append your added objects to the main pointcloud
P      = [savedP addedP]; 
C      = [savedC addedC];

% figure(4); clf;
% samples = randperm(min(size(savedP,2),size(savedP,2)));
% if(numel(samples) > 1)
%     samples = samples(1:min(numel(samples), 1000)); %If your computer can't handle the 3D visualization, reduce this number
% 
%     %visualize your added objects
%     if(numel(addedP) > 0)
%         samples2 = randperm(size(addedP,2));
%         samples2 = samples2(1:400); %If your computer can't handle the 3D visualization, reduce this number
%     end
%     subplot(1,2,1);
%     scatter3(savedP(1,samples),savedP(2,samples),savedP(3,samples),45,mean(savedC(:,samples),1), '.', 'LineWidth', 0.0001)
%     colormap(gray(256)); hold on;
%     %visualize your added objects
%     if(numel(addedP) > 0)   
%         scatter3(addedP(1,samples2),addedP(2,samples2),addedP(3,samples2),45, mean(addedC(:,samples2),1), '.', 'LineWidth', 0.0001)
%         colormap(gray(256)); hold on; 
%     end
%     xlabel('x'); ylabel('y'); zlabel('z');
%     view([0 -90]); ylim([-0.6,0.6]); axis manual;
% 
%     subplot(1,2,2);
%     scatter3(P(1,samples)',P(2,samples)',P(3,samples)',45,mean(C(:,samples),1), '.', 'LineWidth', 0.0001)
%     colormap(gray(256)); hold on;
%     %visualize your added objects
%     if(numel(addedP) > 0)
%         scatter3(addedP(1,samples2),addedP(2,samples2),addedP(3,samples2),45, mean(addedC(:,samples2),1), '.', 'LineWidth', 0.0001)
%     end
%     colormap(gray(256)); hold on; xlabel('x'); ylabel('y'); zlabel('z'); view([0 0]); zlim([0.3,2.2]); axis manual;
% end
%%
%Now let's generate a movie from our scene
savePath = sprintf([datapath1 'video_%03d.avi'], numI*framePeriod);


% CS268 HW4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Here is some sample code for a matlab tool to generate videos directly
% writerObj = VideoWriter(savePath); % Name it.
% writerObj.FrameRate = 15; % How many frames per second - change this to make your video look nice
% open(writerObj); 
% 
% %Let's print out one frame from the origin of our coordinate system
 Pi = P(1:3,:);
 I  = GenerateImageFromPoints(Pi,C);
 figure;
% vl_tightsubplot(1,1,1);
 imagesc(I/255); drawnow;
% frame = getframe(gcf); writeVideo(writerObj,frame);
% path = sprintf([datapath1 'myReprojectedScene.png']);
% imwrite(I/255, path, 'png');
P(2,:) = -P(2,:);
ptCloud = pointCloud(P','Color', uint8(C'));
% pcshow(ptCloud);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
pcwrite(ptCloud, 'ptcloud');
save('G.mat', 'G');
save('P.mat', 'P');
save('C.mat', 'C');
%sample code for writing out the video frames
%Once you've generated an image and put it in I
%for i=1:howeverManyFramesYouWantInYourVideo
%    figure(5); vl_tightsubplot(1,1,1); %Get a nice looking figure 
%    I = GenerateImageFromPoints(Pi,C); % where Pi is the point cloud from
%    the perspective you want to visualize it at
%    the viewpoint you want for image I
%    imagesc(I/255);
%    drawnow;
%    frame = getframe(gcf); %grab the figure
%    writeVideo(writerObj,frame); %write it out
%    path = sprintf([datapath1 '%03d-reProj.png'], i);
%    imwrite(I/255, path, 'png'); %save the image
%    %loop over this with new image I to generate your video
%end
%close(writerObj) %Close the object when you're done

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
