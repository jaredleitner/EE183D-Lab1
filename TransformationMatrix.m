function TransMatrix=TransformationMatrix(theta)
    TransMatrix=rotationZ(theta(1))*translationX(50)*rotationZ(theta(2))*translationX(30)*translationZ(15)*rotationZ(theta(3))*rotationX(pi/2)*rotationZ(theta(4));
end