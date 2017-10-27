function x=map(pos, x1,c2)
obstacle_x =[3, 9];
 obstacle_y =[pos, 9];
% obstacle_y =[3, 9];
goal_x = 10;
goal_y = 10;
Kr = 15;
ki = 10;
kg=4;
xi=linspace(0,12,13)
yi=linspace(0,12,13);
potential_field=zeros(13,13);
% alpha=zeros(13,13);
% fx=zeros(13,13);
% fy=zeros(13,13);
rg=0;
ri=0;
% t1=[0 20];
% xin=1:13;
% yin=1:13;
 
 
for i=1:13
%     fx=0;
%     fy=0;
    for j=1:13
         rg=sqrt((goal_x - xi(i))^2 + (goal_y - yi(j))^2);
        potential_field(i,j) = Kr * rg;
%         fgx=kg*((goal_x - x)/rg);
%         fgy=kg*((goal_y - y)/rg);
        for obs=1:length(obstacle_x)
%             fix=0;
%             fiy=0;

             ri = sqrt((obstacle_x(obs)-xi(i))^2+(obstacle_y(obs)-yi(j))^2);

%ri = sqrt((obstacle_x(obs)-xi(i))^2); %+(obstacle_y(obs)-yi(j))^2);
            potential_field(i,j) = potential_field(i,j) + ki/ri;
%             fix=fix+(ki*((obstacle_x(obs)-x))/ri^2);
%             fiy=fiy+(ki*((obstacle_y(obs)-y))/ri^2);
%               
        end
%          fx(x,y)=fix+fgx;
%          fy(x,y)=fiy+fgy;
%          alpha(x,y)=atan(fy(x,y)/fx(x,y));
%          
    end
%     

end
t1=[0 0.1];
x_robo = x1;
% 
% if(c2 == 1)
%         x_robo = [x1 pos];
% else
%     x_robo = x1;
%     x_robo(13) = pos;
% end
        [t,x]=ode23('bot',t1,x_robo);
        
 


figure(1)
contour([0:12],[0:12],potential_field);
hold on
plot(x(:,1),x(:,3),'b',x(:,5),x(:,7),'r',x(:,9),x(:,11),'g');
% plot(x(:,1),x(:,2));
hold on
% figure(2)
% mesh([0:12],[0:12],potential_field);
% hold on
end