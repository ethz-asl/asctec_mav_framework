function add_eml_compliance(filename)

f_in = fopen(filename);

if(f_in<2)
    error(['couldn''t open file ' filename]);
end
fn_out = [filename '.tmp'];
f_out = fopen(fn_out,'w');

% get first line
l = fgets(f_in);
% write to file and add %#eml
fprintf(f_out, '%s %%#eml\n', l(1:end-1));

% write rest
while(ischar(l))
    l=fgets(f_in);
    fprintf(f_out, '%s', l);
end

fclose(f_in);
fclose(f_out);

eval(['!mv ' fn_out ' ' filename])

