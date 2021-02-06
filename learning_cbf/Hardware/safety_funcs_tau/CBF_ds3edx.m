function ds3edx = CBF_ds3edx(in1,xmin)
%CBF_DS3EDX
%    DS3EDX = CBF_DS3EDX(IN1,XMIN)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    09-Nov-2020 22:06:12

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
t6 = qsf+t4;
t11 = qsf+qsh+qsk+t2+t5;
t7 = qsk+t6;
t8 = sin(t6);
t12 = cos(t11);
t13 = sin(t11);
t14 = t3+t11;
t9 = cos(t7);
t10 = sin(t7);
t15 = cos(t14);
t16 = sin(t14);
t22 = t13.*(2.54e+2./6.25e+2);
t23 = t13.*3.2512e+1;
t24 = t12.*(2.54e+2./6.25e+2);
t17 = t9.*(2.54e+2./6.25e+2);
t18 = t10.*(2.54e+2./6.25e+2);
t19 = t10.*3.2512e+1;
t25 = -t22;
t26 = -t23;
t27 = t16.*3.7336e+1;
t29 = t15.*4.667e-1;
t30 = t16.*4.667e-1;
t33 = dqnsk.*t15.*(-4.667e-1);
t20 = -t18;
t21 = -t19;
t28 = -t27;
t31 = dqnsk.*t29;
t32 = -t30;
t34 = t24+t29;
t35 = dqnsh.*t34;
t36 = dqsf.*t34;
t37 = dqsh.*t34;
t38 = dqsk.*t34;
t40 = t17+t34;
t39 = -t37;
t41 = dqsk.*t40;
t42 = -t41;
ds3edx = [t8.*(-3.7336e+1)+t21+t26+t28+t31+t35+t39+t42-dqsf.*(t40+cos(t6).*4.667e-1),t21+t26+t28+t31+t35+t39+t42-dqsf.*t40,t26+t28+t31+t35-t36-t38+t39,t23+t27+t33-t35+t36+t37+t38,t27+t33-dqnsh.*t15.*4.667e-1+dqsf.*t29+dqsh.*t29+dqsk.*t29,t8.*(-4.667e-1)+t20+t25+t32,t20+t25+t32,t25+t32,t22+t30,t30];