% Find the transform between two perfect point sets.
% [1] Closed-form solution of absolute orientation using unit quaternions

clear

% Random data (1) or sensor data (0)
if 1
    % Model point set
    Np = 500; % Number of points
    m = rand(3, Np);
    
    % 2D Point Set 
    m(3,:) = zeros(1,Np);

    % Data point set
    R = rand(3, 1) * pi; % Rotation
    T = rand(3, 1); % Translation
    S = rand(1, 1); % Scale
    W = 0.00; % Noise

    % transform the points
    r = rotx(R(1)) * roty(R(2)) * rotz(R(3));
    d = S*r*m + repmat(T,1,Np) + W * rand(size(m));

else
    load('SimpleTest1.mat'); % m, d
    
    % 3D points are required for this implementation
    if(size(m,1) == 2)
        m = [m; zeros(1, size(m,2))];
    end
    if(size(d,1) == 2)
        d = [d; zeros(1, size(d,2))];
    end
end


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


[ t, q, s ] = minimize( m, d );
D = s * quatrotate(q',d')' - repmat(t, 1, size(d,2));


% Plot
figure(2)
clf
plot3(0,0,0,'+K');
hold on
plot3(m(1,:), m(2,:), m(3,:), '.b');
plot3(D(1,:), D(2,:), D(3,:), 'Or');
axis equal
grid on
title('Output Point Sets');
legend('Origin', 'Model','Data');
