% Covariance matrix
C = [4 -1; -1 10];
M = [2 -1]';

% Create N normal samples with cov C and mean M
N = 10000;
L = chol(C);
Z = ( repmat(M',N,1) + randn(N,2)*L );

% Plot the samples
figure(1), clf;
scatter(Z(:,1), Z(:,2));
hold on; grid on; axis equal;
scatter(M(1), M(2), 'rx');

% Calculate statistics
Chat = cov(Z)
Mhat = mean(Z)'

% Perform a linear transformation to the data using a rotation matrix
Rd = @(thetad) [cosd(thetad) -sind(thetad); sind(thetad) cosd(thetad)];

% Calculate statistics of rotated data
th = 30;
C2hat = Rd(th)*Chat*Rd(th)'
M2hat = Rd(th)*Mhat

% it's interesting that even after a rotation, the eigenvalues are the same
% which makes sense from a linear operator point of view --> rotation
% matrices are orthogonal and thus don't change the matrix norm.