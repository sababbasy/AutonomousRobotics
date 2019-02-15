h = kOpenPort();
kSetEncoders(h,0,0)
%basic_mov(h);

p1 = [230,50];
p2 = [230,360];
rad = degtorad(90);
kinematics(h,p1,p2,rad);

%odometry();
%kSetSpeed(h, 70,70);
%task_1_1(h,[70,90],90);
m=0.0;
m(6,2) = 0.0;

%test
% for i=1:1:6
%    meanStd = calibrateIR(h,100,2);
%    for j=1:1:2
%    m(i,j)=meanStd(j);
%    end
%    fprintf('pr');
% end
% 
% x=[100,80,60,40,20,0];
% y=m(:,1);
% err=m(:,2);
% errorbar(x,y,err)
% xlim([0 100]);

%obstacle(h,4);







