function [fx, Fx, Q] = Motion(x, T)
    % Motion calculates the predicted state using a coordinated
    % turn motion model, and also calculated the motion model Jacobian
    %
    % Input:
    %   x           [4 x 1] state vector
    %   T           [1 x 1] Sampling time
    %
    % Output:
    %   fx          [4 x 1] motion model evaluated at state x
    %   Fx          [4 x 4] motion model Jacobian evaluated at state x
    %
    % NOTE: the motion model assumes that the state vector x consist of the
    % following states:
    %   px          X-position
    %   py          Y-position
    %   vx          X-velocity
    %   vy          Y-velocity


    x
    fx = [x(1,:);
          x(2,:) ;
          x(3,:);
          x(4,:);];

        
    Fx = [1 0 T  0;
          0 1 0  T;
          0 0 1  0;
          0 0 0  1;];

    Q = diag([((T.^4)./4) ((T.^4)./4) (T.^2)./2 (T.^2)./2 ]);

%     Q = sigma^2*[
%                 T^4/4   0       T^3/2   0;
%                 0       T^4/4   0       T^3/2;
%                 T^3/2   0       T^2     0;
%                 0       T^3/2   0       T^2;
%                 ];


end