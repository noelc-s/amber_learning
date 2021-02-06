function Lfs3 = CBF_Lfs3(in1,xmin)
%CBF_LFS3
%    LFS3 = CBF_LFS3(IN1,XMIN)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    09-Nov-2020 22:01:59

dqnsh = in1(9,:);
dqnsk = in1(10,:);
dqsf = in1(6,:);
dqsh = in1(8,:);
dqsk = in1(7,:);
qnsh = in1(4,:);
qnsk = in1(5,:);
qsf = in1(1,:);
qsh = in1(3,:);
qsk = in1(2,:);
t2 = -qnsh;
t3 = -qnsk;
t4 = pi./2.0;
t5 = pi.*(3.0./2.0);
t6 = qsf+qsk+t4;
t8 = qsf+qsh+qsk+t2+t5;
t7 = sin(t6);
t9 = sin(t8);
t10 = t3+t8;
t11 = sin(t10);
t12 = t7.*(2.54e+2./6.25e+2);
t13 = t9.*(2.54e+2./6.25e+2);
t14 = t11.*4.667e-1;
t15 = t13+t14;
Lfs3 = -dqsk.*(t12+t15)-dqsf.*(t12+t15+sin(qsf+t4).*4.667e-1)+dqnsh.*t15+dqnsk.*t14-dqsh.*t15;
