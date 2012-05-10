% takes a vector of length 3 and converts to 4x4 skew symetric matrix


function qskew = quatskew_sym(q)
eml.inline('always');
%convention: first element is real part:
% qskew = sym('qskew', [4, 4]);
qskew = sym('qskew', 'real');
qskew(4,4) = 0;
qskew(1,2:4)=-q(2:4);
qskew(2,3)=q(4);
qskew(2,4)=-q(3);
qskew(3,4)=q(2);
qskew=qskew-qskew';
qskew=qskew+diag(ones(1,4)*q(1));
