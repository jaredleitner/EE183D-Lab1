function IK=InverseKinematics(theta0,xDesired)
   
    q0 = theta0;    %initial joint space
    x0 = TransformationMatrix(q0)*[0;0;0;1];  %initial operational space
   
    dx = (xDesired - x0)./1000;  %difference between initial x and desired x 
    
    while norm(dx) > 10^-3
        dq = inverseJacobian(q0)*dx;   %step in q
        q0 = q0 + dq;       %updated joint space
        x0 = TransformationMatrix(q0)*[0;0;0;1];  %updated operational space
        dx = (xDesired - x0)./1000;  %updated difference between x and desired x
    end
    IK = q0;  %return updated joint space
end