function [tang,perp]=tangperp_targ(h,start,stop)

% normalized stop-start
npts=length(h); % number of data points
d=stop-start; % vector form start to stop
nd=norm(d); % desired dmovement length
d=d/nd;
d=ones(npts,1)*d; % unit vector in direction of desired movement

% normalized r's
st=ones(npts,1)*start;
r=h-st;	 % hand position relative to starting position
lr=rownorm(r)+0.000001; % distance form start
r=r./(lr*ones(1,3)); %unit vector from start to hand


% dot product to get cos theta (c) the angel between straightline and hand
% position
c=rowsum(r.*d);

% cross product to get sin theta (s)
s=cross(d',r')';
s=s(:,3);

% Get perpendicular and tangential distance to intertarget line
tang=lr.*c;
perp=-lr.*s;
  