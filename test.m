a1=imread('1.png');
a2=imread('2.png');
a3=imread('3.png');
a4=imread('4.png');
a5=imread('5.png');
a6=imread('6.png');
subplot(2,3,1);
imshow(a1);
title('Number 1')
subplot(2,3,2);
imshow(a2)
title('Number 2')
subplot(2,3,3);
imshow(a3)
title('Number 3')
subplot(2,3,4);
imshow(a4)
title('Number 4')
subplot(2,3,5);
imshow(a5)
title('Number 5')
subplot(2,3,6);
imshow(a6)
title('Number 6')
%% clustering algorithm
m=load('frame data.txt');
plot(m(:,1),m(:,2),'ro')
xlabel('X Position [m]')
ylabel('Y Position [m]')
title('Lidar Measurements')
%%
clear all;
y=load('changing.txt');
y=y';
% First simulate system for testing purpose
% in real applications, you will be given the measurement of a system,
% instead of generating them on your own
dt=0.1;
A = [1 0 dt 0;
    0 1 0 dt;
    0 0 1 0;
    0 0 0 1];
C = [1 0 0 0; 
    0 1 0 0];  %two dimensional output
R = 0.1*eye(2);  %covariance for measurement noise
% no deterministic control inputs
Q = 0.1*eye(4);%covariance for process noise
 
x(:,1) = [0 0 0 0]';  %in real application, we may not know the intial state
N = 31;
for k = 1:N  %simulate process for 100 steps
    v(:,k) = sqrt(R)*randn(2,1); %measurement noise (zero mean Gaussian with covariance R)
    w(:,k) = sqrt(Q)*randn(4,1); %process noise (zero mean Gaussian with covariance Q)
    x(:,k+1) = A*x(:,k) +w(:,k); %evolve state (here we don't have control input)    
end
% now design Kalman filter to estimate x(k)
% we are only given output y(:,k) (2-dimensional here)
% in your project, you also need to include input u(k)
 
xhat(:,1) = [-1 -9 0 0]; %randomly pick one initial guess, this is different from the true state at beginning
P{1} = 0.1*eye(4);  
N = 31;
for k = 2:N
    %prediction step
    %first compute predicted state at k (in project, you need to include
    %effect from control
    xPred(:,k) = A*xhat(:,k-1);
    %then update covariance matrix 
    Ppred{k} = A*P{k-1}*A' + Q;
    %measurement update step
    %first compute Kalman gain
    K{k} = Ppred{k}*C'*inv(C*Ppred{k}*C' + R);
    % then do the update
    xhat(:,k) = xPred(:,k)  + K{k}*(y(:,k)-C*xPred(:,k));
    P{k} = (eye(4) - K{k}*C)*Ppred{k};    
end
 
figure;
plot(xhat(1,:),xhat(2,:)); 
hold on;
plot(y(1,:),y(2,:),'ro');
legend('Kalman','Measurements')
xlabel('X Position [m]');
ylabel('Y Position [m]');
title('Tracking Vehicle')
figure;

quiver(xhat(1,:),xhat(2,:),xhat(3,:),xhat(4,:))






















