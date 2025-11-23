


SATx = 416.7;
SATy = -1173.7;
attsat = 0.3;

MAx = -486.5633;
MAy =  780.6364;



dxma = MAx - SATx
dyma = MAy - SATy

RI2Bma = [ cos( attsat ), sin( attsat ); -sin( attsat ), cos( attsat ) ];

dxy_body = RI2Bma * [ dxma; dyma ]

SS = sqrt( dxy_body(2)^2 +  dxy_body(1)^2 )

SS = SS/1000

theta = atan2( dxy_body(2), dxy_body(1) )
theta = wrapToPi( theta )

disp(rad2deg(theta))


