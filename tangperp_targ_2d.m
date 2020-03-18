function [tang,perp]=tangperp_targ(h,start,stop)

% normalized stop-start
npts=length(h);
d=stop-start;
nd=vecnorm(d);
d=d/vecnorm(d);
d=ones(npts,1)*d;

% normalized r's
st=ones(npts,1)*start;
r=h-st+0.0001;	
lr=rownorm(r);
r=r./(rownorm(r)*ones(1,2));


% dot product to get cos theta (c)
c=rowsum(r.*d);

% cross product to get sin theta (s)
s=cross(d',r')';
s=s(:,3);

% Get perpendicular and tangential distance to intertarget line
tang=lr.*c/nd;
perp=-lr.*s;
  