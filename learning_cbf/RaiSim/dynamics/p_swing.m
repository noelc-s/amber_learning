function pfoot2 = p_swing(in1)
%P_SWING
%    PFOOT2 = P_SWING(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    09-Nov-2020 22:00:39

qnsh = in1(4,:);
qnsk = in1(5,:);
qsf = in1(1,:);
qsh = in1(3,:);
qsk = in1(2,:);
t2 = -qnsh;
t3 = -qnsk;
t4 = pi./2.0;
t5 = pi.*(3.0./2.0);
t6 = qsf+t4;
t8 = qsf+qsh+qsk+t2+t5;
t7 = qsk+t6;
t9 = t3+t8;
pfoot2 = [cos(t6).*4.667e-1+cos(t7).*(2.54e+2./6.25e+2)+cos(t8).*(2.54e+2./6.25e+2)+cos(t9).*4.667e-1;sin(t6).*4.667e-1+sin(t7).*(2.54e+2./6.25e+2)+sin(t8).*(2.54e+2./6.25e+2)+sin(t9).*4.667e-1];
