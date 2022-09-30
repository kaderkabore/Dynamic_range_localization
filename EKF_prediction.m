function [x, P] = EKF_prediction(x, P, f, Q)
    % EKF_prediction calculates mean and covariance of predicted state
    %   density using a non-linear Gaussian model.
    %
    % Input:
    %   x           [n x 1] Prior mean
    %   P           [n x n] Prior covariance
    %   f           Motion model function handle
    %               [fx,Fx]=f(x) 
    %               Takes as input x (state), 
    %               Returns fx and Fx, motion model and Jacobian evaluated at x
    %               All other model parameters, such as sample time T,
    %               must be included in the function
    %   Q           [n x n] Process noise covariance
    %   type        String that specifies the type of non-linear filter
    %
    %Output:
    %   x           [n x 1] predicted state mean
    %   P           [n x n] predicted state covariance

    n = size(x,1);
    
    % calculate f(x) and jacobian(f(x),x)
    [fx, dfx] = f(x);
    % predict using first order Taylor expansion
    x = fx;
    P = dfx * P * dfx' + Q;
        

end