clear, clc;

H = eye(2)*2;
g = [-4;-4];
A = [2 4; 5 5];
ubA = [28;50];
lb = [0;0];
ub = [8;8];

x = quadprog(H,g,A,ubA,[],[],lb,ub)