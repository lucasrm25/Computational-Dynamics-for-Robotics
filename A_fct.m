function A = A_fct(x,y,phi,dx,dy,dphi,m,theta,grav,d)
%A_FCT
%    A = A_FCT(X,Y,PHI,DX,DY,DPHI,M,THETA,GRAV,D)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    19-Feb-2020 13:10:44

t2 = cos(phi);
t3 = sin(phi);
t4 = t2.*x;
t5 = t3.*y;
A = reshape([m,0.0,0.0,-t3,0.0,m,0.0,t2,0.0,0.0,theta,-t4-t5,t3,-t2,t4+t5,0.0],[4,4]);