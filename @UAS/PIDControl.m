function yk = PIDControl(Kp, Ki, Kd, ykm1, ek, ekm1, ekm2)
    %PICONTROL Proportional Derivative Control Step Function 
    % On Input:
    %   Kp - Proportional Gain
    %   Ki - Integral Gain
    %   Kd - Derivative Gain
    %   ykm1 - dx1 previous position
    %   ek - dx1 Error between current and desired position
    %   ekm1 - dx1  ek at (k-1) step
    %   ekm2 - dx1  ek at (k-2) step
    % On Output:
    %   yk = dx1 acceleration output
    %   
    yk = ek*(Kp + Ki + Kd) - ekm1*(Kp+2*Kd) + ekm2*Kd + ykm1;
end
