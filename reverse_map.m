function [i] = reverse_map(x,XMIN,XMAX,DIVX);
i = (x-XMIN)/(XMAX-XMAX)*(DIVX -1) +1 ;
