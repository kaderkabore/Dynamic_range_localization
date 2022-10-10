function [hx, Hx, R] = Measurement(x, s1, s2 , s3)
    % MEASUREMENT calculates the bearings from 3 references, located in 
    % s1, s2 and s3, to the position given by the state vector x. Also returns the
    % Jacobian of the model at x.
    %
    % Input:
    %   x           [4 x 1] State vector, the two first element are 2D position
    %   s1          [2 x 1] Sensor position (2D) for reference 1
    %   s2          [2 x 1] Sensor position (2D) for references 2
    %   s3          [2 x 1] Sensor position (2D) for references 3
    %
    % Output:
    %   hx          [3 x 1] measurement vector
    %   Hx          [3 x n] measurement model Jacobian
    %
    % NOTE: the measurement model assumes that in the state vector x, the first
    % two states are X-position and Y-position.


    % initialize outputs with correct sizes
    n  = size(x,1);
    N = size(x,2);
    hx = zeros(2,N);
    Hx = zeros(2,n);

    % calculate readings from the three sensors
    rng1 = norm(x(1:2)-s1); 
    rng2 = norm(x(1:2)-s2); 
    rng3 = norm(x(1:2)-s3); 

    % output is the concatenation of the three readings
    hx(1:3,:) = [rng1; 
                 rng2;
                 rng3];
    
    % jacobian of hx, as calculated using the symbolic toolbox
    Hx(1:3,1:4) = [ (x(1)-s1(1))/rng1 , (x(2)-s1(2))/rng1, 0,0;
                    (x(1)-s2(1))/rng2 , (x(2)-s2(2))/rng2, 0,0;
                    (x(1)-s3(1))/rng3 , (x(2)-s3(2))/rng3, 0,0;];

     sigma_r = 0.1;

    R = [sigma_r^2 0         0;
         0         sigma_r^2 0;
         0         0         sigma_r^2];

end
