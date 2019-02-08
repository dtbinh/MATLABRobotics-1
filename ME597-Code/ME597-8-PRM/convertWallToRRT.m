% %  convert wall coords to env coords for RRT
% function [a,b,c,d,e] = convertWallToRRT(int1, int2, int3, int4)
% a = [int1, int2];
% e = a;
% b = [int1 + int3, int2];
% c = [int1 + int3, int2 + int4];
% d = [int1, int2 + int4];
% end
%  convert wall coords to env coords for RRT
function [a,b,c,d,e] = convertWallToRRT(intArr)
a = [intArr(1), intArr(2)];
e = a;
b = [intArr(1) + intArr(3), intArr(2)];
c = [intArr(1) + intArr(3), intArr(2) + intArr(4)];
d = [intArr(1), intArr(2) + intArr(4)];
end
