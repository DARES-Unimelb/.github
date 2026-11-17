v = ver; fid = fopen('C:\Ash\Projects\DARES_UACS\MATLAB\docs\_verdump.txt','w');
fprintf(fid, 'MATLAB %s\n', version);
fprintf(fid, 'matlabroot %s\n', matlabroot);
for k = 1:numel(v)
   fprintf(fid, '%s | %s\n', v(k).Name, v(k).Version);
end
fclose(fid);
disp('done');
