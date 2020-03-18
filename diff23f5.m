%DIFF23F5 Five point differentiation algorithm with filter
% This program performs a double five point differentiation on data after
% applying a low- or bandpass filter
% Differentiated data array is padded to maintain vector lengths
%       vout = diff23f5( v, ts, fc );
%     --inputs    : v: data matrix
%                   ts: sample time
%                   fc: cut-off frequency
%     --outputs   : v1    : original filtered data
%                   vdot  : differentiated data
%                   vddot : double differentiated data
%     --calls     : n/a
%  Created by Alaa Ahmed
%  Last modified  21-Jun-2006

function        vout = diff23f5( v, ts, fc );

[m,n] = size(v);

%%% Filter data
[B_coef_filter, A_coef_filter] = butter(4,2.0*ts*fc );
for     i=1:n,
   v1(:,i) = filtfilt( B_coef_filter, A_coef_filter, v(:,i) );
end;

%%% First five-point differentiation
vdot(3:m-2,:) = (v1(1:m-4,:)-8*v1(2:m-3,:)+8*v1(4:m-1,:)-v1(5:m,:))/(12.0*ts);
vdot(1,:) = vdot(3,:);
vdot(m-1,:)=vdot(m-2,:);
vdot(m,:) = vdot(m-1,:);

%%% Second five-point differentiation
vddot(3:m-2,:) = (-v1(1:m-4,:)+16*v1(2:m-3,:)-30*v1(3:m-2,:)+16*v1(4:m-1,:)-v1(5:m,:))/(12.0*ts^2);
vddot(1,:) = vddot(3,:);
vddot(m-1,:)=vddot(m-2,:);
vddot(m,:) = vddot(m-1,:);


vout = [ v1 vdot vddot ];
size(vout);

