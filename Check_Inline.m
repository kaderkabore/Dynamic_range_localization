function [isLine] = Check_Inline(pos1,pos2,pos3)
% Check if these 3 points are aligned
x1 = pos1(1);
y1 = pos1(2);
x2 = pos2(1);
y2 = pos2(2);
x3 = pos3(1);
y3 = pos3(2);
isLine = rank([x2-x1, y2-y1; x3-x1, y3-y1]) < 2;