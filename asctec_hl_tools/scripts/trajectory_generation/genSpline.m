function [x x_dot x_ddot t] = genSpline(D, T, N, var, splinedim)

t=0:T:D;
% if(size(var, 2) > 1)
%     error('''var'' has to be a column vector!')
% end



pts = (rand(length(var),N)*2-1) .* var(:,ones(1,N));

F1=splinefit((0:D/(N-1):D), pts, N, splinedim); 
dF1=ppdiff(F1,1);
ddF1=ppdiff(F1,2);
% iF1=fnint(F1);

Ft1     = ppval(F1,t);
dFt1    = ppval(dF1,t);
ddFt1   = ppval(ddF1,t);
% iFt1    = ppval(iF1,t);

x       = Ft1(:,1:end-1);		
x_dot   = dFt1(:,1:end-1);		
x_ddot  = ddFt1(:,1:end-1);
