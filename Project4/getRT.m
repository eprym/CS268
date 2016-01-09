load 'G.mat';
for i = 1 : size(G,2)
    G_last = eye(4);
    for j = 1:i
        G_last = G_last * G{j};
    end
    G_all{i} = G_last;
    R{i} = G_all{i}(1:3, 1:3);
    T{i} = G_all{i}(1:3, 4);
end
save('R.mat', 'R');
save('T.mat', 'T');
save('G_all', 'G_all');