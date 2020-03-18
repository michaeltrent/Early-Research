function[out]=rowsum(in)

if size(in,1)==1, 
  out=in;
else
  out=(sum(in'))';
end