function R = rotatezyx(ang)

% X
c = cos(ang(1));
s = sin(ang(1));
Rx = [1  0   0;
      0  c  -s;
      0  s   c];

% Y
c = cos(ang(2));
s = sin(ang(2));
Ry = [ c  0   s;
       0  1   0;
      -s  0   c];

% Z
c = cos(ang(3));
s = sin(ang(3));
Rz = [ c  -s  0;
       s   c  0;
       0   0  1];

R = Rz*Ry*Rx;   

end