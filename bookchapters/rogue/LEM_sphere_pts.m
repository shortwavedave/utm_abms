function pts = LEM_sphere_pts(center,radius,num_pts)
%

pts = zeros(num_pts,3);

for k = 1:num_pts
    theta = 2*pi*rand;
    x = cos(theta);
    y = sin(theta);
    P = [x;y;0];
    u = [-y;x;0];
    ux = -y;
    uy = x;
    uz = 0;
    
    phi = 2*pi*rand;
    c = cos(phi);
    s = sin(phi);
    R = zeros(3,3);
    R(1,1) = c + ux^2*(1-c);
    R(1,2) = ux*uy*(1-c) - uz*s;
    R(1,3) = ux*uz*(1-c) + uy*s;
    R(2,1) = uy*ux*(1-c) + uz*s;
    R(2,2) = c + uy^2*(1-c);
    R(2,3) = uy*uz*(1-c) - ux*s;
    R(3,1) = uz*ux*(1-c) - uy*s;
    R(3,2) = uz*uy*(1-c) + ux*s;
    R(3,3) = x + uz^2*(1-c);
    
    pt = radius*R*P;
    pts(k,:) = pt;
end
pts(:,1) = pts(:,1) + center(1);
pts(:,2) = pts(:,2) + center(2);
pts(:,3) = pts(:,3) + center(3);
