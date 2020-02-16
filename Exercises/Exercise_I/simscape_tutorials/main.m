L = 20;
H = 1;
W = 1;
rgb = [1 0 0];
rho = 2700;

figure; % Open a new figure
hold on;
plot(out.q); % Plot the pendulum angle
plot(out.w); % Plot the pendulum angular velocity

figure;
plot(out.q.data, out.w.data);


A = [1 1; 1 -1]/2^0.5
det(A)
A^-1


syms H P c Rk M Pk
M = Pk^-1;
P = Rk^-1
UV = [H.'*P*H+M 0; 0 P-P*H*(H.'*P*H+M)^-1*H.'*P]

inv(UV)