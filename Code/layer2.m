function [t,x]=layer2()

clc
Goal=[10;10];
Obs1=[3;3];

Obs2=[9;9];

time=[0 300];

x0=[0;0;0;0;0;0;0;0;0;0;0;0];

[t x]=ode23('bot',time,x0);

plot(x(:,1),x(:,3),'b',x(:,5),x(:,7),'r',x(:,9),x(:,11),'g',Goal(1),Goal(2),'o',Obs1(1),Obs1(2),'x',Obs2(1),Obs2(2),'x')

axis([0 12 0 12])
end