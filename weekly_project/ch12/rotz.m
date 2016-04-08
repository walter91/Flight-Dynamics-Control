% returns a 3d rotation matrix for ccw rotation about z-axis by 'angle' radians
function out = rotz(angle)

  out = [cos(angle), -sin(angle), 0;...
         sin(angle), cos(angle), 0;...
         0,          0,          1;];

end