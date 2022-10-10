function [x, P] = EKF_Update(x, P, y, h)
    % EKF_Update calculates mean and covariance of predicted state
    %   density using a non-linear Gaussian model.
    %
    % Input:
    %   x           [n x 1] Predicted mean
    %   P           [n x n] Predicted covariance
    %   y           [m x 1] measurement vector
    %   h           Measurement model function handle
    %               [hx,Hx,R]=h(x) 
    %               Takes as input x (state), 
    %               Returns hx and Hx, measurement model and Jacobian evaluated at x
    %               Function must include all model parameters for the particular model, 
    %               such as sensor position for some models.
    %               R [m x m] Measurement noise covariance
    %   type        String that specifies the type of non-linear filter
    %
    % Output:
    %   x           [n x 1] updated state mean
    %   P           [n x n] updated state covariance
    
    n = size(x,1);
    
    % calculate h(x) and jacobian(h(x),x)
    [hx, dhx, R] = h(x);
    % calculate inovation covariance and kalman gain
    S = dhx * P * dhx' + R;
    K = P * dhx' / S;
    
    % update
    x = x + K * ( y - hx );
    P = P - K * S * K';   
        

end