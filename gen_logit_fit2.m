function [y] = gen_logit_fit2(x,param,t,dat) 

% This is a generalized logistic function 
% Inputs: param(1) = A  lower asymptote
%         param(2) = K  Upper asymptote
%         param(3) = M The inflection point
%         x(1) = B  The growth rate
%         x(2) = Q  Scales the growth
%         x(3) = M  The inflection point
%         x(4) = v  The

%         t = vector of time...

%         dat       the data you are trying to fit

% if dat is row vec make it column

if size(dat,2) > 1 && numel(dat) > 2
    dat = dat';
end

% same for t

if size(t,2) > 1 && numel(t) > 2
    t = t';
end


func = @(x,t,dat,param) (param(1)+(param(2)-param(1))./((1+x(2)*exp(-(x(1)*(t-param(3)))).^(1/x(3))))) - dat;
options = optimset('TolX',1e-2000,'TolFun',1e-100,'MaxFunEvals',1000, 'MaxIter',1000);
 x =lsqnonlin(@(x) func(x,t,dat,param),x,[],[],options);
 y = [param x];
 
 fit = @(x,t) x(1)+(x(2)-x(1))./((1+x(5)*exp(-(x(4)*(t-x(3)))).^(1/x(6))));
 
 figure('color','w')
 plot(t,dat,'bo')
 hold on
 plot(t,fit(y,t),'k','linewidth',2)
end