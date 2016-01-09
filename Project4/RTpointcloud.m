function [P_rt,C_rt] = RTpointcloud(P,C,G)
  P_rt = pinv(G) * [P; ones(1,size(P,2))];
  P_rt(4,:) = [];
  C_rt = C;
