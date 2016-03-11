% compute_trim

function [X,U] = compute_trim(SYS,Va,gamma,R)

DX0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va/R*cos(gamma); 0; 0; 0];
IDX = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

X0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
IX = [];
U0 = [0; 0; 0; 1];
IU = [];
Y0 = [Va; gamma; 0];
IY = [1,3];

[X,U,~,DX] = trim(SYS,X0,U0,Y0,IX,IU,IY,DX0,IDX);

norm(DX(3:end)-DX0(3:end))

end