% Find the transform between two perfect point sets.
% [1] Closed-form solution of absolute orientation using unit quaternions


% Model point set
N = 500; % Number of points
m = rand(3, N);

% Data point set
R = rand(3, 1) * pi; % Rotation
T = rand(3, 1); % Translation
S = rand(1, 1); % Scale
W = 0.01; % Noise

% transform the points
r = rotx(R(1)) * roty(R(2)) * rotz(R(3));
d = S*r*m + repmat(T,1,N) + W * rand(size(m));

% Plot
figure(1)
clf
plot3(0,0,0,'+K');
hold on
plot3(m(1,:), m(2,:), m(3,:), '.b');
plot3(d(1,:), d(2,:), d(3,:), '.r');
axis equal
grid on
title('Input Point Sets');
legend('Origin', 'Model','Data');


%
% Find the transform
%


% Translation (Centroid)
um = mean(m,2); 
ud = mean(d,2);

m1 = m - repmat(um,1,N);
d1 = d - repmat(ud,1,N);

% Plot
figure(2)
clf
plot3(0,0,0,'+K');
hold on
plot3(m1(1,:), m1(2,:), m1(3,:), '.b');
plot3(d1(1,:), d1(2,:), d1(3,:), '.r');
axis equal
grid on
title('Centered Point Sets');
legend('Origin', 'Model','Data');



% Scale
s = sqrt(sum(d1.^2) / sum(m1.^2));

d2 = (1/s)*d1;

% Plot
figure(3)
clf
plot3(0,0,0,'+K');
hold on
plot3(m1(1,:), m1(2,:), m1(3,:), '.b');
plot3(d2(1,:), d2(2,:), d2(3,:), '.r');
axis equal
grid on
title('Scaled Point Sets');
legend('Origin', 'Model','Data');



% Rotation

% 'pseudo cross correlation matrix' 
M = zeros(3);
for i = 1:N
    M = M + m1(:,i) * d2(:,i)';
end

% N Matrix
N = zeros(4);

t = M - M';
a = [t(2,3); t(3,1); t(1,2)]; 

N(1  , :  ) = [trace(M) a'];
N(2:4, 1  ) = a;
N(2:4, 2:4) = M + M' - trace(M)*eye(3);

% Find eigenvector with max eigenvalue
[v, e] = eig(N);
[~, I] = max(max(e));
Q = v(:,I);

% Perform the rotation
d3 = quatrotate(Q',d2')';


% Plot
figure(4)
clf
plot3(0,0,0,'+K');
hold on
plot3(m1(1,:), m1(2,:), m1(3,:), '.b');
plot3(d3(1,:), d3(2,:), d3(3,:), 'Or');
axis equal
grid on
title('Rotated Point Sets');
legend('Origin', 'Model','Data');



