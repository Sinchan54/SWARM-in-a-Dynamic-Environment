strt = 1;
fin = 150;
for c2 = strt:fin

tob=[0 0.1];

if ( c2 == 1 )
        x1=[0 0 0 0 0 0 0 0 0 0 0 0 0];
        %tob1=[0 1];
        xob1 = [3 3];
        [t,xb]=ode23('obj1_1',tob,xob1);
        pos_obj= xb(size(xb,1),1);
        
        xrobo = map(pos_obj, x1,c2);
        %[t,x]=ode23('robo_1',t1,x1);

else
    
  
        
        xob1 = xb(size(xb,1),:);
        [t,xb]=ode23('obj1_1',tob,xob1);
        pos_obj= xb(size(xb,1),1);
        x1 = xrobo(size(xrobo,1),:);
        xrobo = map(pos_obj, x1,c2);
        %[t,x]=ode23('robo_2',t1,x2);
    
end
    
    
    
end