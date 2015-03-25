function [ R_return,T_return ] = RANSAC( X , Y ,iter,thr,mininlier)
%UNTITLED4 Summary of this function goes here
%Detailed explanation goes here
%delcare R and T
R_return = [0,0;0,0];
T_return = [0;0];
%calculate the centroid
Xc = sum(X,2)/size(X,2);
Yc = sum(Y,2)/size(Y,2);
%calculate the respective coordinates
Xr = X - repmat(Xc,1,size(X,2));
Yr = Y - repmat(Yc,1,size(X,2));
errmin = -1;
valid_final = 0;
% --------------------------------------------------------------------
%              RANSAC 1 to remove mathces with large errors
% --------------------------------------------------------------------

for i = 1:100
   index = randi([1 size(X,2)],3,1); % generate three indexs for points
   X_select = [Xr(:,index(1)) Xr(:,index(2)) Xr(:,index(3))]; %pick points 
   Y_select = [Yr(:,index(1)) Yr(:,index(2)) Yr(:,index(3))];
   R = Y_select * X_select' * pinv(X_select * X_select'); %compute transformation
   T = Yc - R * Xc;
   T = repmat(T,1,size(X,2));
   Y_est = R * X + T;
   Y_diff = sum((Y - Y_est).^2);
   valid = Y_diff < 5000;
   inlier_num = sum(valid);
   if(inlier_num > (0.5 * size(X,2)))
       err = sum(valid.*Y_diff);
       if(err < errmin || errmin < 0)
            valid_final = valid;
            errmin = err;
       end
   end 
end
% remove mathces with large errors
valid_final = repmat(valid_final,2,1);
sum(sum(valid_final))
X_valid = X;
Y_valid = Y;
X_valid(:,valid_final(1,:) == 0) = [];
Y_valid(:,valid_final(1,:) == 0) = [];

X = X_valid;
Y = Y_valid;
   
% --------------------------------------------------------------------
%               RANSAC 2 Redo RANSAC with filtered matches
% --------------------------------------------------------------------

%calculate the centroid
Xc = sum(X,2)/size(X,2);
Yc = sum(Y,2)/size(Y,2);
%calculate the respective coordinates
Xr = X - repmat(Xc,1,size(X,2));
Yr = Y - repmat(Yc,1,size(X,2));
errmin = -1;
valid_final = 0;
for i = 1:iter
   index = randi([1 size(X,2)],3,1); % generate three indexs for points
   X_select = [Xr(:,index(1)) Xr(:,index(2)) Xr(:,index(3))]; %pick points 
   Y_select = [Yr(:,index(1)) Yr(:,index(2)) Yr(:,index(3))];
   R = Y_select * X_select' * pinv(X_select * X_select'); %compute transformation
   T = Yc - R * Xc;
   T = repmat(T,1,size(X,2));
   Y_est = R * X + T;
   Y_diff = sum((Y - Y_est).^2);
   valid = Y_diff < thr;
   inlier_num = sum(valid);
   if(inlier_num >mininlier)
       err = sum(valid.*Y_diff);
       if(err < errmin || errmin < 0)
            valid_final = valid;
            errmin = err;
            
       end
   end 
end
    sum(sum(valid_final))
    %refit process using best inliers group
    valid_final = repmat(valid_final,2,1);
    X_valid =X;
    Y_valid =Y;
    X_valid(:,valid_final(1,:) == 0) = [];
    Y_valid(:,valid_final(1,:) == 0) = [];
    Xc = sum(X_valid,2)/size(X_valid,2);
    Yc = sum(Y_valid,2)/size(Y_valid,2);
    Xr = X_valid - repmat(Xc,1,size(X_valid,2));
    Yr = Y_valid - repmat(Yc,1,size(Y_valid,2));
    
    R_return = Yr * Xr' * pinv(Xr  * Xr')
    T_return = Yc - R_return * Xc
    
end

