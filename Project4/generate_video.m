function generate_video(Iend,insertnum)

  %load('record.mat');
  close all;
  clc;
  load('P.mat');
  load('C.mat');
  load('R.mat');
  load('T.mat');
  P(2,:) = -P(2,:);
  P_all = P;
  C_all = C;
  clearvars -except Istart Iend replace insertnum R T G_all P_all C_all record
  close all

  Istart = 1;
  datapath1 = 'Data/images/';
  datapath2 = 'Data/';
  datapath3 = 'Data/depths/';
  datapath4 = 'Data/video/';
  Depths    = dir([datapath3, '*.mat']);

  savePath = sprintf([datapath4 'video_%03d_%03d.avi'], Iend,insertnum);
  % lambda = 1/(insertnum+1);

  writerObj = VideoWriter(savePath); % Name it.
  writerObj.FrameRate = 15; % How many frames per second - change this to make your video look nice
  open(writerObj);

%   framePeriod = 1;
%   %Depth image loading
%   depthPath = [datapath3 Depths(Istart).name];
%   temp      = load(depthPath);
%   DIm1      = temp.depth;
%   %color image loading
%   path = sprintf([datapath1 '%03d.png'], Istart);
%   I1   = double(imresize(imread(path), 1));
%   numI = Iend - Istart;

  %load ('model1.mat');
  load('G_all.mat');

  G_all{1} = eye(4); %homogeneous form of transformation
  R{1} = eye(3); %List of rotations and translations
  T{1} = [0;0;0];

  % CS268 HW4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %Here is some sample code for a matlab tool to generate videos directly

  for i= Istart : Iend
    if i ==Istart
      P_rt = P_all;
      C_rt = C_all;
    else
      [P_rt,C_rt] = RTpointcloud(P_all,C_all,G_all{i});
    end

    I  = GenerateImageFromPoints(P_rt,C_rt);
    pause(0.0001);
%     figure;
    imagesc(I/255);
    fprintf('processing image %02d ...\n',i);
    % title(['The ',num2str(Istart),'-th image generated from point cloud']);
    drawnow;
    frame = getframe(gcf); writeVideo(writerObj,frame);
    path = sprintf([datapath4 num2str(i) '_00.png']);
    imwrite(I/255, path, 'png');

    if i ~=Iend
      for j = 1 : insertnum
        lambda = 1/(insertnum+1) *j;
        lambda = 1 - lambda;
        Ttemp = lambda * T{i} + (1 - lambda) * T{i+1};
        Rtemp = lambda * R{i} + (1 - lambda) * R{i+1};
        [U S V] = svd(Rtemp);
        Rtemp = U * V';
        

        Gtemp = [Rtemp Ttemp; 0,0,0,1 ];
        [P_rt,C_rt] = RTpointcloud(P_all,C_all,Gtemp);
        I  = GenerateImageFromPoints(P_rt,C_rt);
        pause(0.0001);
%         figure;
        imagesc(I/255);
        fprintf('generating new frame %d.%d ...\n',i,j);
        % title(['The ',num2str(Istart),'-th image generated from point cloud']);
        drawnow;
        frame = getframe(gcf); writeVideo(writerObj,frame);
        path = sprintf([datapath4 num2str(i) '_' num2str(j) '.png']);
        imwrite(I/255, path, 'png');

      end
    end
  end
  close(writerObj) %Close the object when you're done
end
