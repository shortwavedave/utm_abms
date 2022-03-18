function trajn = LEM_noisy_traj(traj,ratio,speed,del_t)
%

ZERO_THRESH = 0.001;

[num_pts,~] = size(traj);
trajn = traj;
T = zeros(3,3);
for p = 1:num_pts-1
    pt1 = traj(p,1:3);
    pt2 = traj(p+1,1:3);
    dist12 = norm(pt2-pt1);
    if dist12<ZERO_THRESH
        trajn(p+1,1:3) = pt1 + 0.01*randn(1,3);
    else
        radius = dist12*ratio;
        done = 0;
        while done==0
            pu = [-radius;-radius] + 2*radius*rand(2,1);
            if norm(pu)<=radius
                done = 1;
            end
        end
        pu = [pu;0;1];
        dir2 = pt2;
        dir2 = dir2/norm(dir2);
        z = [0;0;1];
        theta = acos(dot(z,dir2));
        d_perp = cross(z,dir2);
        T = LEM_rot_3D(d_perp,theta);
        T(1:3,4) = pt2;
        pc = T*pu;
        pc = pc(1:3)';
        dist1c = norm(pc-pt1);
        if ratio==0
            v = dist1c;
        else
            v = dist1c - min(dist1c/2,randn);
        end
        dir1c = pc - pt1;
        dir1c = dir1c/norm(dir1c);
        trajn(p+1,1:3) = pt1 + v*dir1c*speed*del_t;
    end
end

tch = 0;
