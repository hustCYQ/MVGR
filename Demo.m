% configure
% The number of point clouds
n = 20;
% The number of sampling points of each point cloud
m = 3;
points = textread('bunny.txt');
points = points';

% R£ºRotate Matrix
% t£ºTrans Vector
P = struct;
% Rand generate n point clouds
for i = 1:n
    P(i).R = [1 0 0; 0 1 0;0 0 1];
    P(i).t = [0 0 0]';
    sita = rand();
    rou = rand(3,1);
    rou = rou/norm(rou);
    t = rand(3,1);
    R = axis2R(sita,rou);
    P(i).points = R*points+t;

end
% lamda
lamda = 0.15;
mu = m - sqrt(m*m-lamda);
% iter
iters = 10;
dist = struct;
colors = rand(n,3);

for iter = 1:iters
    iter
    xlim([0 20])
    ylim([0 20])
    zlim([0 20])
    figure('color',[1 1 1]);
    hold on;
    for i = 1:n
        X = (P(i).R*P(i).points+P(i).t)';
        scatter3(X(:,1),X(:,2),X(:,3),[5],colors(i,:),'.');
        view([2.144453576476345e+02,-37.704684601321517])
    end

    title( "iter: "+num2str(iter-1))
    pause(1)
    
    
    mean_L1 = 0;
    f = zeros(n*(n-1)/2*m*3,1);
    J = zeros(n*(n-1)/2*m*3,6*n);
    idx = randperm(362261);
    idx = idx(1:m);

    for i = 1:n
        P(i).sampoints = P(i).points(:,idx);
        P(i).sampoints = P(i).R*P(i).sampoints+P(i).t;
    end
    

    cnt = 1;
    for i = 1:n
        for j = i+1:n
            dist_E = P(i).sampoints-P(j).sampoints;
            dist_mean = sum(dist_E,2);
            dist_mean = dist_mean/m;
            
            dist = P(i).sampoints-P(j).sampoints;
            dist = dist - lamda*dist_mean;
            f((cnt-1)*3*m+1:cnt*3*m) = dist;
            cnt = cnt + 1;
            mean_L1 = mean_L1 + abs(dist_mean);
        end
    end
    mean_L1 = mean_L1/(n*(n-1))*2;
    mean_L1 = sum(abs(mean_L1))
    if(mean_L1<1e-4)
        break
    end


    
%   cal J and D
    cnt = 1;
    for i = 1:n
        for j = i+1:n
            D = zeros(m*3,6);
            for k = 1:m
                D((k-1)*3+1:k*3,1:3) = eye(3);
                RIN = P(i).R*P(i).sampoints(:,k)+P(i).t;
                D((k-1)*3+1:k*3,4:6) = skew(-RIN);
            end
            J((cnt-1)*3*m+1:cnt*3*m,(i-1)*6+1:i*6) = D;
            
            for k = 1:m
                D((k-1)*3+1:k*3,1:3) = eye(3);
                RIN = P(j).R*P(j).sampoints(:,k)+P(j).t;
                D((k-1)*3+1:k*3,4:6) = skew(-RIN);
            end
            J((cnt-1)*3*m+1:cnt*3*m,(j-1)*6+1:j*6) = -D;
            cnt = cnt + 1;
        end
    end
    md = J;
    A = eye(m) - mu*ones(m,m)/m;
    A = kron(A,[1 0 0;0 1 0;0 0 1]);
    A = kron(A,eye(n*(n-1)/2));
    J = (A*J)';
    d_kesi = pinv(J*J')*(-J)*f;
    if(iter>50)
        d_kesi = d_kesi*0.1;
    else if(iter>100)
            d_kesi = d_kesi*0.01;
        end
    end
    
    
    for i = 1:n
        T = SE3.exp(d_kesi((i-1)*6+1:i*6));
        T = double(T);
        delta_R = T(1:3,1:3);
        delta_t = T(1:3,4);
        P(i).R = delta_R * P(i).R;
        P(i).t = P(i).t + delta_t;    
    end
    

end





