h = kOpenPort();
kSetEncoders(h,0,0)

p1 = [230,50];
p2 = [230,360];
rad = degtorad(90);

avoid_obstacle(h,p1,p2,rad);
