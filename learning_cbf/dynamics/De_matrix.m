function De = De_matrix(in1)
%DE_MATRIX
%    DE = DE_MATRIX(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    27-Oct-2020 16:55:36

qnsh = in1(4,:);
qnsk = in1(5,:);
qsf = in1(1,:);
qsh = in1(3,:);
qsk = in1(2,:);
t2 = cos(qnsk);
t3 = cos(qsf);
t4 = cos(qsh);
t5 = cos(qsk);
t6 = sin(qsf);
t7 = qsf+qsk;
t8 = qsh+qsk;
t13 = -qnsh;
t14 = -qnsk;
t15 = -qsh;
t16 = -qsk;
t9 = cos(t7);
t10 = cos(t8);
t11 = qsh+t7;
t12 = sin(t7);
t19 = qnsh+t15;
t22 = t8+t13;
t28 = qnsh+qnsk-t8;
t33 = t3.*8.461467014;
t34 = t6.*8.461467014;
t45 = t4.*1.9744964417536e-1;
t46 = t4.*9.872482208768e-2;
t53 = t5.*3.1998554405116;
t59 = t2.*9.081028320000001e-1;
t60 = t2.*1.816205664;
t17 = cos(t11);
t18 = sin(t11);
t20 = cos(t19);
t21 = qnsk+t19;
t24 = cos(t22);
t25 = t11+t13;
t29 = cos(t28);
t35 = -t33;
t36 = -t34;
t37 = t9.*6.856343348;
t38 = t12.*6.856343348;
t55 = t10.*1.1337321473504e-1;
t61 = -t60;
t62 = -t59;
t77 = t59+5.78077891925e-1;
t23 = cos(t21);
t26 = cos(t25);
t27 = sin(t25);
t30 = t14+t25;
t39 = t17.*2.429252512e-1;
t40 = t18.*2.429252512e-1;
t41 = -t37;
t42 = -t38;
t51 = t20.*2.1988323629056;
t52 = t20.*4.3976647258112;
t57 = t24.*2.5250862789568;
t71 = t29.*1.0428434835;
t78 = t62-5.78077891925e-1;
t81 = t61-2.7634769149026;
t31 = cos(t30);
t32 = sin(t30);
t43 = -t39;
t44 = -t40;
t47 = t26.*5.410512704;
t48 = t27.*5.410512704;
t54 = -t51;
t56 = -t52;
t58 = -t57;
t63 = t23.*9.081028320000001e-1;
t64 = t23.*1.816205664;
t72 = -t71;
t49 = -t47;
t50 = -t48;
t65 = t31.*2.234505;
t66 = t32.*2.234505;
t67 = -t64;
t68 = -t63;
t84 = t63+t78;
t85 = t51+t63+t81;
t69 = -t65;
t70 = -t66;
t75 = t43+t47+t65;
t76 = t44+t48+t66;
t86 = t71+t84;
t87 = t46+t54+t60+t68+2.852013641262184;
t88 = t57+t71+t85;
t90 = t45+t53+t55+t56+t58+t60+t67+t72+5.545783727509384;
t73 = t49+t69;
t74 = t50+t70;
t79 = t41+t75;
t80 = t42+t76;
t89 = t55+t58+t72+t87;
t82 = t35+t79;
t83 = t36+t80;
De = reshape([t5.*6.3997108810232+t10.*2.2674642947008e-1-t24.*5.0501725579136-t29.*2.085686967+t45+t56+t60+t67+9.831432893886457,t90,t89,t88,t86,t82,t83,t90,t45+t56+t60+t67+5.972102361305344,t87,t85,t84,t79,t80,t89,t87,t60+3.278332275058144,t81,t78,t75,t76,t88,t85,t81,t60+3.18979554869856,t77,t73,t74,t86,t84,t78,t77,1.00439652572096,t69,t70,t82,t79,t75,t73,t69,1.813042e+1,0.0,t83,t80,t76,t74,t70,0.0,1.813042e+1],[7,7]);