function generateTrajectoryFile(tp, filename, t_first_wp)

f = fopen(filename, 'w');

if(nargin == 3)
    tp(end, 1) = t_first_wp;
end

for i=1:size(tp, 2)
    fprintf(f, '%f ', tp(:,i)');
    fprintf(f,'\n');
end

fclose(f);

