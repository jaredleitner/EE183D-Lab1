function J_inv= inverseJacobian(theta)
    theta1=theta(1);
    theta2=theta(2);
    
    %inverse jacobian that was calculated in findingInverseJacobian.m
    J_inv=[ - ((30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2))*(50*cos(theta1) + 30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2)) + (30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1))*(50*sin(theta1) + 30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1)))*(30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1)) - ((50*cos(theta1) + 30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2))^2 + (50*sin(theta1) + 30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1))^2)*(50*sin(theta1) + 30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1)), ((50*cos(theta1) + 30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2))^2 + (50*sin(theta1) + 30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1))^2)*(50*cos(theta1) + 30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2)) + ((30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2))*(50*cos(theta1) + 30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2)) + (30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1))*(50*sin(theta1) + 30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1)))*(30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2)), 0, 0;
                                   - ((30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2))^2 + (30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1))^2)*(30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1)) - ((30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2))*(50*cos(theta1) + 30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2)) + (30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1))*(50*sin(theta1) + 30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1)))*(50*sin(theta1) + 30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1)),                                   ((30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2))*(50*cos(theta1) + 30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2)) + (30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1))*(50*sin(theta1) + 30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1)))*(50*cos(theta1) + 30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2)) + ((30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2))^2 + (30*cos(theta1)*sin(theta2) + 30*cos(theta2)*sin(theta1))^2)*(30*cos(theta1)*cos(theta2) - 30*sin(theta1)*sin(theta2)), 0, 0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0, 0, 0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0, 0, 0];
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
end