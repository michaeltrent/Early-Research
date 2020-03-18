function [xaxis,yaxis]=batchplotformat(v,gapsize,gaplocation)

%v is a matrix of y axis data (each column is a different dataset)
%gapsize is a scalar denoting the size in #batches of the gap b/n phases
%gaplocation is a row vector specifying the locations
%transpose v if needed
if length(v) ~= size(v,1); v=v'; end
gap = NaN*zeros(gapsize,1);
gaplocations=[0 gaplocation];
xaxis=[1:size(v,1)+gapsize*size(gaplocation,2)]';

yaxis=[];
for i=1:size(gaplocation,2)
    yaxis=[yaxis; v(gaplocations(i)+1:gaplocations(i+1),:); gap];
end
yaxis = [yaxis; v(gaplocations(end)+1:size(v,1),:)];        
       