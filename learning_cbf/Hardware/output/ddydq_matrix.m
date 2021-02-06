function ddydq = ddydq_matrix(in1,in2,in3,in4)
%DDYDQ_MATRIX
%    DDYDQ = DDYDQ_MATRIX(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    09-Nov-2020 22:02:44

alpha1_1 = in4(1);
alpha1_2 = in4(5);
alpha1_3 = in4(9);
alpha1_4 = in4(13);
alpha1_5 = in4(17);
alpha2_1 = in4(2);
alpha2_2 = in4(6);
alpha2_3 = in4(10);
alpha2_4 = in4(14);
alpha2_5 = in4(18);
alpha3_1 = in4(3);
alpha3_2 = in4(7);
alpha3_3 = in4(11);
alpha3_4 = in4(15);
alpha3_5 = in4(19);
alpha4_1 = in4(4);
alpha4_2 = in4(8);
alpha4_3 = in4(12);
alpha4_4 = in4(16);
alpha4_5 = in4(20);
c1 = in2(:,1);
c2 = in2(:,2);
c3 = in2(:,3);
c4 = in2(:,4);
c5 = in2(:,5);
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
thetamp1 = in3(:,1);
thetamp2 = in3(:,2);
t2 = c4.*qnsh;
t3 = c5.*qnsk;
t4 = c1.*qsf;
t5 = c3.*qsh;
t6 = c2.*qsk;
t7 = c1.^2;
t8 = c2.^2;
t9 = c3.^2;
t10 = c4.^2;
t11 = c5.^2;
t12 = -thetamp2;
t13 = t12+thetamp1;
t18 = t2+t3+t4+t5+t6+t12;
t14 = 1.0./t13;
t19 = t18.^2;
t15 = t14.^2;
t16 = t14.^3;
t20 = t14.*t18;
t17 = t15.^2;
t21 = t20-1.0;
t22 = t21.^2;
t23 = alpha1_3.*c1.*c2.*t17.*t19.*1.2e+1;
t24 = alpha1_3.*c1.*c3.*t17.*t19.*1.2e+1;
t25 = alpha1_3.*c1.*c4.*t17.*t19.*1.2e+1;
t26 = alpha1_3.*c2.*c3.*t17.*t19.*1.2e+1;
t27 = alpha1_5.*c1.*c2.*t17.*t19.*1.2e+1;
t28 = alpha1_3.*c1.*c5.*t17.*t19.*1.2e+1;
t29 = alpha1_3.*c2.*c4.*t17.*t19.*1.2e+1;
t30 = alpha1_5.*c1.*c3.*t17.*t19.*1.2e+1;
t31 = alpha1_3.*c2.*c5.*t17.*t19.*1.2e+1;
t32 = alpha1_3.*c3.*c4.*t17.*t19.*1.2e+1;
t33 = alpha1_5.*c1.*c4.*t17.*t19.*1.2e+1;
t34 = alpha1_5.*c2.*c3.*t17.*t19.*1.2e+1;
t35 = alpha1_3.*c3.*c5.*t17.*t19.*1.2e+1;
t36 = alpha1_5.*c1.*c5.*t17.*t19.*1.2e+1;
t37 = alpha1_5.*c2.*c4.*t17.*t19.*1.2e+1;
t38 = alpha1_3.*c4.*c5.*t17.*t19.*1.2e+1;
t39 = alpha1_5.*c2.*c5.*t17.*t19.*1.2e+1;
t40 = alpha1_5.*c3.*c4.*t17.*t19.*1.2e+1;
t41 = alpha1_5.*c3.*c5.*t17.*t19.*1.2e+1;
t42 = alpha1_5.*c4.*c5.*t17.*t19.*1.2e+1;
t43 = alpha2_3.*c1.*c2.*t17.*t19.*1.2e+1;
t44 = alpha2_3.*c1.*c3.*t17.*t19.*1.2e+1;
t45 = alpha2_3.*c1.*c4.*t17.*t19.*1.2e+1;
t46 = alpha2_3.*c2.*c3.*t17.*t19.*1.2e+1;
t47 = alpha2_5.*c1.*c2.*t17.*t19.*1.2e+1;
t48 = alpha1_4.*c1.*c2.*t17.*t19.*2.4e+1;
t49 = alpha2_3.*c1.*c5.*t17.*t19.*1.2e+1;
t50 = alpha2_3.*c2.*c4.*t17.*t19.*1.2e+1;
t51 = alpha2_5.*c1.*c3.*t17.*t19.*1.2e+1;
t52 = alpha1_4.*c1.*c3.*t17.*t19.*2.4e+1;
t53 = alpha2_3.*c2.*c5.*t17.*t19.*1.2e+1;
t54 = alpha2_3.*c3.*c4.*t17.*t19.*1.2e+1;
t55 = alpha2_5.*c1.*c4.*t17.*t19.*1.2e+1;
t56 = alpha2_5.*c2.*c3.*t17.*t19.*1.2e+1;
t57 = alpha1_4.*c1.*c4.*t17.*t19.*2.4e+1;
t58 = alpha1_4.*c2.*c3.*t17.*t19.*2.4e+1;
t59 = alpha2_3.*c3.*c5.*t17.*t19.*1.2e+1;
t60 = alpha2_5.*c1.*c5.*t17.*t19.*1.2e+1;
t61 = alpha2_5.*c2.*c4.*t17.*t19.*1.2e+1;
t62 = alpha1_4.*c1.*c5.*t17.*t19.*2.4e+1;
t63 = alpha1_4.*c2.*c4.*t17.*t19.*2.4e+1;
t64 = alpha2_3.*c4.*c5.*t17.*t19.*1.2e+1;
t65 = alpha2_5.*c2.*c5.*t17.*t19.*1.2e+1;
t66 = alpha2_5.*c3.*c4.*t17.*t19.*1.2e+1;
t67 = alpha1_4.*c2.*c5.*t17.*t19.*2.4e+1;
t68 = alpha1_4.*c3.*c4.*t17.*t19.*2.4e+1;
t69 = alpha2_5.*c3.*c5.*t17.*t19.*1.2e+1;
t70 = alpha1_4.*c3.*c5.*t17.*t19.*2.4e+1;
t71 = alpha2_5.*c4.*c5.*t17.*t19.*1.2e+1;
t72 = alpha1_4.*c4.*c5.*t17.*t19.*2.4e+1;
t73 = alpha3_3.*c1.*c2.*t17.*t19.*1.2e+1;
t74 = alpha3_3.*c1.*c3.*t17.*t19.*1.2e+1;
t75 = alpha3_3.*c1.*c4.*t17.*t19.*1.2e+1;
t76 = alpha3_3.*c2.*c3.*t17.*t19.*1.2e+1;
t77 = alpha3_5.*c1.*c2.*t17.*t19.*1.2e+1;
t78 = alpha2_4.*c1.*c2.*t17.*t19.*2.4e+1;
t79 = alpha3_3.*c1.*c5.*t17.*t19.*1.2e+1;
t80 = alpha3_3.*c2.*c4.*t17.*t19.*1.2e+1;
t81 = alpha3_5.*c1.*c3.*t17.*t19.*1.2e+1;
t82 = alpha2_4.*c1.*c3.*t17.*t19.*2.4e+1;
t83 = alpha3_3.*c2.*c5.*t17.*t19.*1.2e+1;
t84 = alpha3_3.*c3.*c4.*t17.*t19.*1.2e+1;
t85 = alpha3_5.*c1.*c4.*t17.*t19.*1.2e+1;
t86 = alpha3_5.*c2.*c3.*t17.*t19.*1.2e+1;
t87 = alpha2_4.*c1.*c4.*t17.*t19.*2.4e+1;
t88 = alpha2_4.*c2.*c3.*t17.*t19.*2.4e+1;
t89 = alpha3_3.*c3.*c5.*t17.*t19.*1.2e+1;
t90 = alpha3_5.*c1.*c5.*t17.*t19.*1.2e+1;
t91 = alpha3_5.*c2.*c4.*t17.*t19.*1.2e+1;
t92 = alpha2_4.*c1.*c5.*t17.*t19.*2.4e+1;
t93 = alpha2_4.*c2.*c4.*t17.*t19.*2.4e+1;
t94 = alpha3_3.*c4.*c5.*t17.*t19.*1.2e+1;
t95 = alpha3_5.*c2.*c5.*t17.*t19.*1.2e+1;
t96 = alpha3_5.*c3.*c4.*t17.*t19.*1.2e+1;
t97 = alpha2_4.*c2.*c5.*t17.*t19.*2.4e+1;
t98 = alpha2_4.*c3.*c4.*t17.*t19.*2.4e+1;
t99 = alpha3_5.*c3.*c5.*t17.*t19.*1.2e+1;
t100 = alpha2_4.*c3.*c5.*t17.*t19.*2.4e+1;
t101 = alpha3_5.*c4.*c5.*t17.*t19.*1.2e+1;
t102 = alpha2_4.*c4.*c5.*t17.*t19.*2.4e+1;
t103 = alpha4_3.*c1.*c2.*t17.*t19.*1.2e+1;
t104 = alpha4_3.*c1.*c3.*t17.*t19.*1.2e+1;
t105 = alpha4_3.*c1.*c4.*t17.*t19.*1.2e+1;
t106 = alpha4_3.*c2.*c3.*t17.*t19.*1.2e+1;
t107 = alpha4_5.*c1.*c2.*t17.*t19.*1.2e+1;
t108 = alpha3_4.*c1.*c2.*t17.*t19.*2.4e+1;
t109 = alpha4_3.*c1.*c5.*t17.*t19.*1.2e+1;
t110 = alpha4_3.*c2.*c4.*t17.*t19.*1.2e+1;
t111 = alpha4_5.*c1.*c3.*t17.*t19.*1.2e+1;
t112 = alpha3_4.*c1.*c3.*t17.*t19.*2.4e+1;
t113 = alpha4_3.*c2.*c5.*t17.*t19.*1.2e+1;
t114 = alpha4_3.*c3.*c4.*t17.*t19.*1.2e+1;
t115 = alpha4_5.*c1.*c4.*t17.*t19.*1.2e+1;
t116 = alpha4_5.*c2.*c3.*t17.*t19.*1.2e+1;
t117 = alpha3_4.*c1.*c4.*t17.*t19.*2.4e+1;
t118 = alpha3_4.*c2.*c3.*t17.*t19.*2.4e+1;
t119 = alpha4_3.*c3.*c5.*t17.*t19.*1.2e+1;
t120 = alpha4_5.*c1.*c5.*t17.*t19.*1.2e+1;
t121 = alpha4_5.*c2.*c4.*t17.*t19.*1.2e+1;
t122 = alpha3_4.*c1.*c5.*t17.*t19.*2.4e+1;
t123 = alpha3_4.*c2.*c4.*t17.*t19.*2.4e+1;
t124 = alpha4_3.*c4.*c5.*t17.*t19.*1.2e+1;
t125 = alpha4_5.*c2.*c5.*t17.*t19.*1.2e+1;
t126 = alpha4_5.*c3.*c4.*t17.*t19.*1.2e+1;
t127 = alpha3_4.*c2.*c5.*t17.*t19.*2.4e+1;
t128 = alpha3_4.*c3.*c4.*t17.*t19.*2.4e+1;
t129 = alpha4_5.*c3.*c5.*t17.*t19.*1.2e+1;
t130 = alpha3_4.*c3.*c5.*t17.*t19.*2.4e+1;
t131 = alpha4_5.*c4.*c5.*t17.*t19.*1.2e+1;
t132 = alpha3_4.*c4.*c5.*t17.*t19.*2.4e+1;
t133 = alpha4_4.*c1.*c2.*t17.*t19.*2.4e+1;
t134 = alpha4_4.*c1.*c3.*t17.*t19.*2.4e+1;
t135 = alpha4_4.*c1.*c4.*t17.*t19.*2.4e+1;
t136 = alpha4_4.*c2.*c3.*t17.*t19.*2.4e+1;
t137 = alpha4_4.*c1.*c5.*t17.*t19.*2.4e+1;
t138 = alpha4_4.*c2.*c4.*t17.*t19.*2.4e+1;
t139 = alpha4_4.*c2.*c5.*t17.*t19.*2.4e+1;
t140 = alpha4_4.*c3.*c4.*t17.*t19.*2.4e+1;
t141 = alpha4_4.*c3.*c5.*t17.*t19.*2.4e+1;
t142 = alpha4_4.*c4.*c5.*t17.*t19.*2.4e+1;
t343 = alpha1_2.*c1.*c2.*t16.*t18.*t21.*2.4e+1;
t344 = alpha1_2.*c1.*c3.*t16.*t18.*t21.*2.4e+1;
t345 = alpha1_2.*c1.*c4.*t16.*t18.*t21.*2.4e+1;
t346 = alpha1_2.*c2.*c3.*t16.*t18.*t21.*2.4e+1;
t347 = alpha1_4.*c1.*c2.*t16.*t18.*t21.*2.4e+1;
t348 = alpha1_2.*c1.*c5.*t16.*t18.*t21.*2.4e+1;
t349 = alpha1_2.*c2.*c4.*t16.*t18.*t21.*2.4e+1;
t350 = alpha1_4.*c1.*c3.*t16.*t18.*t21.*2.4e+1;
t351 = alpha1_2.*c2.*c5.*t16.*t18.*t21.*2.4e+1;
t352 = alpha1_2.*c3.*c4.*t16.*t18.*t21.*2.4e+1;
t353 = alpha1_4.*c1.*c4.*t16.*t18.*t21.*2.4e+1;
t354 = alpha1_4.*c2.*c3.*t16.*t18.*t21.*2.4e+1;
t355 = alpha1_2.*c3.*c5.*t16.*t18.*t21.*2.4e+1;
t356 = alpha1_4.*c1.*c5.*t16.*t18.*t21.*2.4e+1;
t357 = alpha1_4.*c2.*c4.*t16.*t18.*t21.*2.4e+1;
t358 = alpha1_2.*c4.*c5.*t16.*t18.*t21.*2.4e+1;
t359 = alpha1_4.*c2.*c5.*t16.*t18.*t21.*2.4e+1;
t360 = alpha1_4.*c3.*c4.*t16.*t18.*t21.*2.4e+1;
t361 = alpha1_4.*c3.*c5.*t16.*t18.*t21.*2.4e+1;
t362 = alpha1_4.*c4.*c5.*t16.*t18.*t21.*2.4e+1;
t363 = alpha2_2.*c1.*c2.*t16.*t18.*t21.*2.4e+1;
t364 = alpha2_2.*c1.*c3.*t16.*t18.*t21.*2.4e+1;
t365 = alpha2_2.*c1.*c4.*t16.*t18.*t21.*2.4e+1;
t366 = alpha2_2.*c2.*c3.*t16.*t18.*t21.*2.4e+1;
t367 = alpha2_4.*c1.*c2.*t16.*t18.*t21.*2.4e+1;
t368 = alpha2_2.*c1.*c5.*t16.*t18.*t21.*2.4e+1;
t369 = alpha2_2.*c2.*c4.*t16.*t18.*t21.*2.4e+1;
t370 = alpha2_4.*c1.*c3.*t16.*t18.*t21.*2.4e+1;
t371 = alpha2_2.*c2.*c5.*t16.*t18.*t21.*2.4e+1;
t372 = alpha2_2.*c3.*c4.*t16.*t18.*t21.*2.4e+1;
t373 = alpha2_4.*c1.*c4.*t16.*t18.*t21.*2.4e+1;
t374 = alpha2_4.*c2.*c3.*t16.*t18.*t21.*2.4e+1;
t375 = alpha2_2.*c3.*c5.*t16.*t18.*t21.*2.4e+1;
t376 = alpha2_4.*c1.*c5.*t16.*t18.*t21.*2.4e+1;
t377 = alpha2_4.*c2.*c4.*t16.*t18.*t21.*2.4e+1;
t378 = alpha2_2.*c4.*c5.*t16.*t18.*t21.*2.4e+1;
t379 = alpha2_4.*c2.*c5.*t16.*t18.*t21.*2.4e+1;
t380 = alpha2_4.*c3.*c4.*t16.*t18.*t21.*2.4e+1;
t381 = alpha2_4.*c3.*c5.*t16.*t18.*t21.*2.4e+1;
t382 = alpha2_4.*c4.*c5.*t16.*t18.*t21.*2.4e+1;
t383 = alpha3_2.*c1.*c2.*t16.*t18.*t21.*2.4e+1;
t384 = alpha3_2.*c1.*c3.*t16.*t18.*t21.*2.4e+1;
t385 = alpha3_2.*c1.*c4.*t16.*t18.*t21.*2.4e+1;
t386 = alpha3_2.*c2.*c3.*t16.*t18.*t21.*2.4e+1;
t387 = alpha3_4.*c1.*c2.*t16.*t18.*t21.*2.4e+1;
t388 = alpha3_2.*c1.*c5.*t16.*t18.*t21.*2.4e+1;
t389 = alpha3_2.*c2.*c4.*t16.*t18.*t21.*2.4e+1;
t390 = alpha3_4.*c1.*c3.*t16.*t18.*t21.*2.4e+1;
t391 = alpha3_2.*c2.*c5.*t16.*t18.*t21.*2.4e+1;
t392 = alpha3_2.*c3.*c4.*t16.*t18.*t21.*2.4e+1;
t393 = alpha3_4.*c1.*c4.*t16.*t18.*t21.*2.4e+1;
t394 = alpha3_4.*c2.*c3.*t16.*t18.*t21.*2.4e+1;
t395 = alpha1_3.*c1.*c2.*t16.*t18.*t21.*4.8e+1;
t396 = alpha3_2.*c3.*c5.*t16.*t18.*t21.*2.4e+1;
t397 = alpha3_4.*c1.*c5.*t16.*t18.*t21.*2.4e+1;
t398 = alpha3_4.*c2.*c4.*t16.*t18.*t21.*2.4e+1;
t399 = alpha1_3.*c1.*c3.*t16.*t18.*t21.*4.8e+1;
t400 = alpha3_2.*c4.*c5.*t16.*t18.*t21.*2.4e+1;
t401 = alpha3_4.*c2.*c5.*t16.*t18.*t21.*2.4e+1;
t402 = alpha3_4.*c3.*c4.*t16.*t18.*t21.*2.4e+1;
t403 = alpha1_3.*c1.*c4.*t16.*t18.*t21.*4.8e+1;
t404 = alpha1_3.*c2.*c3.*t16.*t18.*t21.*4.8e+1;
t405 = alpha3_4.*c3.*c5.*t16.*t18.*t21.*2.4e+1;
t406 = alpha1_3.*c1.*c5.*t16.*t18.*t21.*4.8e+1;
t407 = alpha1_3.*c2.*c4.*t16.*t18.*t21.*4.8e+1;
t408 = alpha3_4.*c4.*c5.*t16.*t18.*t21.*2.4e+1;
t409 = alpha1_3.*c2.*c5.*t16.*t18.*t21.*4.8e+1;
t410 = alpha1_3.*c3.*c4.*t16.*t18.*t21.*4.8e+1;
t411 = alpha1_3.*c3.*c5.*t16.*t18.*t21.*4.8e+1;
t412 = alpha4_2.*c1.*c2.*t16.*t18.*t21.*2.4e+1;
t413 = alpha1_3.*c4.*c5.*t16.*t18.*t21.*4.8e+1;
t414 = alpha4_2.*c1.*c3.*t16.*t18.*t21.*2.4e+1;
t415 = alpha4_2.*c1.*c4.*t16.*t18.*t21.*2.4e+1;
t416 = alpha4_2.*c2.*c3.*t16.*t18.*t21.*2.4e+1;
t417 = alpha4_4.*c1.*c2.*t16.*t18.*t21.*2.4e+1;
t418 = alpha4_2.*c1.*c5.*t16.*t18.*t21.*2.4e+1;
t419 = alpha4_2.*c2.*c4.*t16.*t18.*t21.*2.4e+1;
t420 = alpha4_4.*c1.*c3.*t16.*t18.*t21.*2.4e+1;
t421 = alpha4_2.*c2.*c5.*t16.*t18.*t21.*2.4e+1;
t422 = alpha4_2.*c3.*c4.*t16.*t18.*t21.*2.4e+1;
t423 = alpha4_4.*c1.*c4.*t16.*t18.*t21.*2.4e+1;
t424 = alpha4_4.*c2.*c3.*t16.*t18.*t21.*2.4e+1;
t425 = alpha2_3.*c1.*c2.*t16.*t18.*t21.*4.8e+1;
t426 = alpha4_2.*c3.*c5.*t16.*t18.*t21.*2.4e+1;
t427 = alpha4_4.*c1.*c5.*t16.*t18.*t21.*2.4e+1;
t428 = alpha4_4.*c2.*c4.*t16.*t18.*t21.*2.4e+1;
t429 = alpha2_3.*c1.*c3.*t16.*t18.*t21.*4.8e+1;
t430 = alpha4_2.*c4.*c5.*t16.*t18.*t21.*2.4e+1;
t431 = alpha4_4.*c2.*c5.*t16.*t18.*t21.*2.4e+1;
t432 = alpha4_4.*c3.*c4.*t16.*t18.*t21.*2.4e+1;
t433 = alpha2_3.*c1.*c4.*t16.*t18.*t21.*4.8e+1;
t434 = alpha2_3.*c2.*c3.*t16.*t18.*t21.*4.8e+1;
t435 = alpha4_4.*c3.*c5.*t16.*t18.*t21.*2.4e+1;
t436 = alpha2_3.*c1.*c5.*t16.*t18.*t21.*4.8e+1;
t437 = alpha2_3.*c2.*c4.*t16.*t18.*t21.*4.8e+1;
t438 = alpha4_4.*c4.*c5.*t16.*t18.*t21.*2.4e+1;
t439 = alpha2_3.*c2.*c5.*t16.*t18.*t21.*4.8e+1;
t440 = alpha2_3.*c3.*c4.*t16.*t18.*t21.*4.8e+1;
t441 = alpha2_3.*c3.*c5.*t16.*t18.*t21.*4.8e+1;
t442 = alpha2_3.*c4.*c5.*t16.*t18.*t21.*4.8e+1;
t443 = alpha3_3.*c1.*c2.*t16.*t18.*t21.*4.8e+1;
t444 = alpha3_3.*c1.*c3.*t16.*t18.*t21.*4.8e+1;
t445 = alpha3_3.*c1.*c4.*t16.*t18.*t21.*4.8e+1;
t446 = alpha3_3.*c2.*c3.*t16.*t18.*t21.*4.8e+1;
t447 = alpha3_3.*c1.*c5.*t16.*t18.*t21.*4.8e+1;
t448 = alpha3_3.*c2.*c4.*t16.*t18.*t21.*4.8e+1;
t449 = alpha3_3.*c2.*c5.*t16.*t18.*t21.*4.8e+1;
t450 = alpha3_3.*c3.*c4.*t16.*t18.*t21.*4.8e+1;
t451 = alpha3_3.*c3.*c5.*t16.*t18.*t21.*4.8e+1;
t452 = alpha3_3.*c4.*c5.*t16.*t18.*t21.*4.8e+1;
t453 = alpha4_3.*c1.*c2.*t16.*t18.*t21.*4.8e+1;
t454 = alpha4_3.*c1.*c3.*t16.*t18.*t21.*4.8e+1;
t455 = alpha4_3.*c1.*c4.*t16.*t18.*t21.*4.8e+1;
t456 = alpha4_3.*c2.*c3.*t16.*t18.*t21.*4.8e+1;
t457 = alpha4_3.*c1.*c5.*t16.*t18.*t21.*4.8e+1;
t458 = alpha4_3.*c2.*c4.*t16.*t18.*t21.*4.8e+1;
t459 = alpha4_3.*c2.*c5.*t16.*t18.*t21.*4.8e+1;
t460 = alpha4_3.*c3.*c4.*t16.*t18.*t21.*4.8e+1;
t461 = alpha4_3.*c3.*c5.*t16.*t18.*t21.*4.8e+1;
t462 = alpha4_3.*c4.*c5.*t16.*t18.*t21.*4.8e+1;
t143 = -t48;
t144 = -t52;
t145 = -t57;
t146 = -t58;
t147 = -t62;
t148 = -t63;
t149 = -t67;
t150 = -t68;
t151 = -t70;
t152 = -t72;
t153 = -t78;
t154 = -t82;
t155 = -t87;
t156 = -t88;
t157 = -t92;
t158 = -t93;
t159 = -t97;
t160 = -t98;
t161 = -t100;
t162 = -t102;
t163 = -t108;
t164 = -t112;
t165 = -t117;
t166 = -t118;
t167 = -t122;
t168 = -t123;
t169 = -t127;
t170 = -t128;
t171 = -t130;
t172 = -t132;
t173 = -t133;
t174 = -t134;
t175 = -t135;
t176 = -t136;
t177 = -t137;
t178 = -t138;
t179 = -t139;
t180 = -t140;
t181 = -t141;
t182 = -t142;
t183 = alpha1_1.*c1.*c2.*t15.*t22.*1.2e+1;
t184 = alpha1_1.*c1.*c3.*t15.*t22.*1.2e+1;
t185 = alpha1_1.*c1.*c4.*t15.*t22.*1.2e+1;
t186 = alpha1_1.*c2.*c3.*t15.*t22.*1.2e+1;
t187 = alpha1_3.*c1.*c2.*t15.*t22.*1.2e+1;
t188 = alpha1_1.*c1.*c5.*t15.*t22.*1.2e+1;
t189 = alpha1_1.*c2.*c4.*t15.*t22.*1.2e+1;
t190 = alpha1_3.*c1.*c3.*t15.*t22.*1.2e+1;
t191 = alpha1_1.*c2.*c5.*t15.*t22.*1.2e+1;
t192 = alpha1_1.*c3.*c4.*t15.*t22.*1.2e+1;
t193 = alpha1_3.*c1.*c4.*t15.*t22.*1.2e+1;
t194 = alpha1_3.*c2.*c3.*t15.*t22.*1.2e+1;
t195 = alpha1_1.*c3.*c5.*t15.*t22.*1.2e+1;
t196 = alpha1_3.*c1.*c5.*t15.*t22.*1.2e+1;
t197 = alpha1_3.*c2.*c4.*t15.*t22.*1.2e+1;
t198 = alpha1_1.*c4.*c5.*t15.*t22.*1.2e+1;
t199 = alpha1_3.*c2.*c5.*t15.*t22.*1.2e+1;
t200 = alpha1_3.*c3.*c4.*t15.*t22.*1.2e+1;
t201 = alpha1_3.*c3.*c5.*t15.*t22.*1.2e+1;
t202 = alpha1_3.*c4.*c5.*t15.*t22.*1.2e+1;
t203 = alpha2_1.*c1.*c2.*t15.*t22.*1.2e+1;
t204 = alpha2_1.*c1.*c3.*t15.*t22.*1.2e+1;
t205 = alpha2_1.*c1.*c4.*t15.*t22.*1.2e+1;
t206 = alpha2_1.*c2.*c3.*t15.*t22.*1.2e+1;
t207 = alpha2_3.*c1.*c2.*t15.*t22.*1.2e+1;
t208 = alpha1_2.*c1.*c2.*t15.*t22.*2.4e+1;
t209 = alpha2_1.*c1.*c5.*t15.*t22.*1.2e+1;
t210 = alpha2_1.*c2.*c4.*t15.*t22.*1.2e+1;
t211 = alpha2_3.*c1.*c3.*t15.*t22.*1.2e+1;
t212 = alpha1_2.*c1.*c3.*t15.*t22.*2.4e+1;
t213 = alpha2_1.*c2.*c5.*t15.*t22.*1.2e+1;
t214 = alpha2_1.*c3.*c4.*t15.*t22.*1.2e+1;
t215 = alpha2_3.*c1.*c4.*t15.*t22.*1.2e+1;
t216 = alpha2_3.*c2.*c3.*t15.*t22.*1.2e+1;
t217 = alpha1_2.*c1.*c4.*t15.*t22.*2.4e+1;
t218 = alpha1_2.*c2.*c3.*t15.*t22.*2.4e+1;
t219 = alpha2_1.*c3.*c5.*t15.*t22.*1.2e+1;
t220 = alpha2_3.*c1.*c5.*t15.*t22.*1.2e+1;
t221 = alpha2_3.*c2.*c4.*t15.*t22.*1.2e+1;
t222 = alpha1_2.*c1.*c5.*t15.*t22.*2.4e+1;
t223 = alpha1_2.*c2.*c4.*t15.*t22.*2.4e+1;
t224 = alpha2_1.*c4.*c5.*t15.*t22.*1.2e+1;
t225 = alpha2_3.*c2.*c5.*t15.*t22.*1.2e+1;
t226 = alpha2_3.*c3.*c4.*t15.*t22.*1.2e+1;
t227 = alpha1_2.*c2.*c5.*t15.*t22.*2.4e+1;
t228 = alpha1_2.*c3.*c4.*t15.*t22.*2.4e+1;
t229 = alpha2_3.*c3.*c5.*t15.*t22.*1.2e+1;
t230 = alpha1_2.*c3.*c5.*t15.*t22.*2.4e+1;
t231 = alpha2_3.*c4.*c5.*t15.*t22.*1.2e+1;
t232 = alpha1_2.*c4.*c5.*t15.*t22.*2.4e+1;
t233 = alpha3_1.*c1.*c2.*t15.*t22.*1.2e+1;
t234 = alpha3_1.*c1.*c3.*t15.*t22.*1.2e+1;
t235 = alpha3_1.*c1.*c4.*t15.*t22.*1.2e+1;
t236 = alpha3_1.*c2.*c3.*t15.*t22.*1.2e+1;
t237 = alpha3_3.*c1.*c2.*t15.*t22.*1.2e+1;
t238 = alpha2_2.*c1.*c2.*t15.*t22.*2.4e+1;
t239 = alpha3_1.*c1.*c5.*t15.*t22.*1.2e+1;
t240 = alpha3_1.*c2.*c4.*t15.*t22.*1.2e+1;
t241 = alpha3_3.*c1.*c3.*t15.*t22.*1.2e+1;
t242 = alpha2_2.*c1.*c3.*t15.*t22.*2.4e+1;
t243 = alpha3_1.*c2.*c5.*t15.*t22.*1.2e+1;
t244 = alpha3_1.*c3.*c4.*t15.*t22.*1.2e+1;
t245 = alpha3_3.*c1.*c4.*t15.*t22.*1.2e+1;
t246 = alpha3_3.*c2.*c3.*t15.*t22.*1.2e+1;
t247 = alpha2_2.*c1.*c4.*t15.*t22.*2.4e+1;
t248 = alpha2_2.*c2.*c3.*t15.*t22.*2.4e+1;
t249 = alpha3_1.*c3.*c5.*t15.*t22.*1.2e+1;
t250 = alpha3_3.*c1.*c5.*t15.*t22.*1.2e+1;
t251 = alpha3_3.*c2.*c4.*t15.*t22.*1.2e+1;
t252 = alpha2_2.*c1.*c5.*t15.*t22.*2.4e+1;
t253 = alpha2_2.*c2.*c4.*t15.*t22.*2.4e+1;
t254 = alpha3_1.*c4.*c5.*t15.*t22.*1.2e+1;
t255 = alpha3_3.*c2.*c5.*t15.*t22.*1.2e+1;
t256 = alpha3_3.*c3.*c4.*t15.*t22.*1.2e+1;
t257 = alpha2_2.*c2.*c5.*t15.*t22.*2.4e+1;
t258 = alpha2_2.*c3.*c4.*t15.*t22.*2.4e+1;
t259 = alpha3_3.*c3.*c5.*t15.*t22.*1.2e+1;
t260 = alpha2_2.*c3.*c5.*t15.*t22.*2.4e+1;
t261 = alpha3_3.*c4.*c5.*t15.*t22.*1.2e+1;
t262 = alpha2_2.*c4.*c5.*t15.*t22.*2.4e+1;
t263 = alpha4_1.*c1.*c2.*t15.*t22.*1.2e+1;
t264 = alpha4_1.*c1.*c3.*t15.*t22.*1.2e+1;
t265 = alpha4_1.*c1.*c4.*t15.*t22.*1.2e+1;
t266 = alpha4_1.*c2.*c3.*t15.*t22.*1.2e+1;
t267 = alpha4_3.*c1.*c2.*t15.*t22.*1.2e+1;
t268 = alpha3_2.*c1.*c2.*t15.*t22.*2.4e+1;
t269 = alpha4_1.*c1.*c5.*t15.*t22.*1.2e+1;
t270 = alpha4_1.*c2.*c4.*t15.*t22.*1.2e+1;
t271 = alpha4_3.*c1.*c3.*t15.*t22.*1.2e+1;
t272 = alpha3_2.*c1.*c3.*t15.*t22.*2.4e+1;
t273 = alpha4_1.*c2.*c5.*t15.*t22.*1.2e+1;
t274 = alpha4_1.*c3.*c4.*t15.*t22.*1.2e+1;
t275 = alpha4_3.*c1.*c4.*t15.*t22.*1.2e+1;
t276 = alpha4_3.*c2.*c3.*t15.*t22.*1.2e+1;
t277 = alpha3_2.*c1.*c4.*t15.*t22.*2.4e+1;
t278 = alpha3_2.*c2.*c3.*t15.*t22.*2.4e+1;
t279 = alpha4_1.*c3.*c5.*t15.*t22.*1.2e+1;
t280 = alpha4_3.*c1.*c5.*t15.*t22.*1.2e+1;
t281 = alpha4_3.*c2.*c4.*t15.*t22.*1.2e+1;
t282 = alpha3_2.*c1.*c5.*t15.*t22.*2.4e+1;
t283 = alpha3_2.*c2.*c4.*t15.*t22.*2.4e+1;
t284 = alpha4_1.*c4.*c5.*t15.*t22.*1.2e+1;
t285 = alpha4_3.*c2.*c5.*t15.*t22.*1.2e+1;
t286 = alpha4_3.*c3.*c4.*t15.*t22.*1.2e+1;
t287 = alpha3_2.*c2.*c5.*t15.*t22.*2.4e+1;
t288 = alpha3_2.*c3.*c4.*t15.*t22.*2.4e+1;
t289 = alpha4_3.*c3.*c5.*t15.*t22.*1.2e+1;
t290 = alpha3_2.*c3.*c5.*t15.*t22.*2.4e+1;
t291 = alpha4_3.*c4.*c5.*t15.*t22.*1.2e+1;
t292 = alpha3_2.*c4.*c5.*t15.*t22.*2.4e+1;
t293 = alpha4_2.*c1.*c2.*t15.*t22.*2.4e+1;
t294 = alpha4_2.*c1.*c3.*t15.*t22.*2.4e+1;
t295 = alpha4_2.*c1.*c4.*t15.*t22.*2.4e+1;
t296 = alpha4_2.*c2.*c3.*t15.*t22.*2.4e+1;
t297 = alpha4_2.*c1.*c5.*t15.*t22.*2.4e+1;
t298 = alpha4_2.*c2.*c4.*t15.*t22.*2.4e+1;
t299 = alpha4_2.*c2.*c5.*t15.*t22.*2.4e+1;
t300 = alpha4_2.*c3.*c4.*t15.*t22.*2.4e+1;
t301 = alpha4_2.*c3.*c5.*t15.*t22.*2.4e+1;
t302 = alpha4_2.*c4.*c5.*t15.*t22.*2.4e+1;
t463 = -t343;
t464 = -t344;
t465 = -t345;
t466 = -t346;
t467 = -t347;
t468 = -t348;
t469 = -t349;
t470 = -t350;
t471 = -t351;
t472 = -t352;
t473 = -t353;
t474 = -t354;
t475 = -t355;
t476 = -t356;
t477 = -t357;
t478 = -t358;
t479 = -t359;
t480 = -t360;
t481 = -t361;
t482 = -t362;
t483 = -t363;
t484 = -t364;
t485 = -t365;
t486 = -t366;
t487 = -t367;
t488 = -t368;
t489 = -t369;
t490 = -t370;
t491 = -t371;
t492 = -t372;
t493 = -t373;
t494 = -t374;
t495 = -t375;
t496 = -t376;
t497 = -t377;
t498 = -t378;
t499 = -t379;
t500 = -t380;
t501 = -t381;
t502 = -t382;
t503 = -t383;
t504 = -t384;
t505 = -t385;
t506 = -t386;
t507 = -t387;
t508 = -t388;
t509 = -t389;
t510 = -t390;
t511 = -t391;
t512 = -t392;
t513 = -t393;
t514 = -t394;
t515 = -t396;
t516 = -t397;
t517 = -t398;
t518 = -t400;
t519 = -t401;
t520 = -t402;
t521 = -t405;
t522 = -t408;
t523 = -t412;
t524 = -t414;
t525 = -t415;
t526 = -t416;
t527 = -t417;
t528 = -t418;
t529 = -t419;
t530 = -t420;
t531 = -t421;
t532 = -t422;
t533 = -t423;
t534 = -t424;
t535 = -t426;
t536 = -t427;
t537 = -t428;
t538 = -t430;
t539 = -t431;
t540 = -t432;
t541 = -t435;
t542 = -t438;
t303 = -t208;
t304 = -t212;
t305 = -t217;
t306 = -t218;
t307 = -t222;
t308 = -t223;
t309 = -t227;
t310 = -t228;
t311 = -t230;
t312 = -t232;
t313 = -t238;
t314 = -t242;
t315 = -t247;
t316 = -t248;
t317 = -t252;
t318 = -t253;
t319 = -t257;
t320 = -t258;
t321 = -t260;
t322 = -t262;
t323 = -t268;
t324 = -t272;
t325 = -t277;
t326 = -t278;
t327 = -t282;
t328 = -t283;
t329 = -t287;
t330 = -t288;
t331 = -t290;
t332 = -t292;
t333 = -t293;
t334 = -t294;
t335 = -t295;
t336 = -t296;
t337 = -t297;
t338 = -t298;
t339 = -t299;
t340 = -t300;
t341 = -t301;
t342 = -t302;
t543 = t23+t27+t143+t183+t187+t303+t395+t463+t467;
t544 = t24+t30+t144+t184+t190+t304+t399+t464+t470;
t545 = t25+t33+t145+t185+t193+t305+t403+t465+t473;
t546 = t26+t34+t146+t186+t194+t306+t404+t466+t474;
t547 = t28+t36+t147+t188+t196+t307+t406+t468+t476;
t548 = t29+t37+t148+t189+t197+t308+t407+t469+t477;
t549 = t31+t39+t149+t191+t199+t309+t409+t471+t479;
t550 = t32+t40+t150+t192+t200+t310+t410+t472+t480;
t551 = t35+t41+t151+t195+t201+t311+t411+t475+t481;
t552 = t38+t42+t152+t198+t202+t312+t413+t478+t482;
t553 = t43+t47+t153+t203+t207+t313+t425+t483+t487;
t554 = t44+t51+t154+t204+t211+t314+t429+t484+t490;
t555 = t45+t55+t155+t205+t215+t315+t433+t485+t493;
t556 = t46+t56+t156+t206+t216+t316+t434+t486+t494;
t557 = t49+t60+t157+t209+t220+t317+t436+t488+t496;
t558 = t50+t61+t158+t210+t221+t318+t437+t489+t497;
t559 = t53+t65+t159+t213+t225+t319+t439+t491+t499;
t560 = t54+t66+t160+t214+t226+t320+t440+t492+t500;
t561 = t59+t69+t161+t219+t229+t321+t441+t495+t501;
t562 = t64+t71+t162+t224+t231+t322+t442+t498+t502;
t563 = t73+t77+t163+t233+t237+t323+t443+t503+t507;
t564 = t74+t81+t164+t234+t241+t324+t444+t504+t510;
t565 = t75+t85+t165+t235+t245+t325+t445+t505+t513;
t566 = t76+t86+t166+t236+t246+t326+t446+t506+t514;
t567 = t79+t90+t167+t239+t250+t327+t447+t508+t516;
t568 = t80+t91+t168+t240+t251+t328+t448+t509+t517;
t569 = t83+t95+t169+t243+t255+t329+t449+t511+t519;
t570 = t84+t96+t170+t244+t256+t330+t450+t512+t520;
t571 = t89+t99+t171+t249+t259+t331+t451+t515+t521;
t572 = t94+t101+t172+t254+t261+t332+t452+t518+t522;
t573 = t103+t107+t173+t263+t267+t333+t453+t523+t527;
t574 = t104+t111+t174+t264+t271+t334+t454+t524+t530;
t575 = t105+t115+t175+t265+t275+t335+t455+t525+t533;
t576 = t106+t116+t176+t266+t276+t336+t456+t526+t534;
t577 = t109+t120+t177+t269+t280+t337+t457+t528+t536;
t578 = t110+t121+t178+t270+t281+t338+t458+t529+t537;
t579 = t113+t125+t179+t273+t285+t339+t459+t531+t539;
t580 = t114+t126+t180+t274+t286+t340+t460+t532+t540;
t581 = t119+t129+t181+t279+t289+t341+t461+t535+t541;
t582 = t124+t131+t182+t284+t291+t342+t462+t538+t542;
ddydq = reshape([-dqnsh.*t545-dqnsk.*t547-dqsh.*t544-dqsk.*t543-dqsf.*(alpha1_1.*t7.*t15.*t22.*1.2e+1-alpha1_2.*t7.*t15.*t22.*2.4e+1+alpha1_3.*t7.*t17.*t19.*1.2e+1+alpha1_3.*t7.*t15.*t22.*1.2e+1-alpha1_4.*t7.*t17.*t19.*2.4e+1+alpha1_5.*t7.*t17.*t19.*1.2e+1-alpha1_2.*t7.*t16.*t18.*t21.*2.4e+1+alpha1_3.*t7.*t16.*t18.*t21.*4.8e+1-alpha1_4.*t7.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t555-dqnsk.*t557-dqsh.*t554-dqsk.*t553-dqsf.*(alpha2_1.*t7.*t15.*t22.*1.2e+1-alpha2_2.*t7.*t15.*t22.*2.4e+1+alpha2_3.*t7.*t17.*t19.*1.2e+1+alpha2_3.*t7.*t15.*t22.*1.2e+1-alpha2_4.*t7.*t17.*t19.*2.4e+1+alpha2_5.*t7.*t17.*t19.*1.2e+1-alpha2_2.*t7.*t16.*t18.*t21.*2.4e+1+alpha2_3.*t7.*t16.*t18.*t21.*4.8e+1-alpha2_4.*t7.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t565-dqnsk.*t567-dqsh.*t564-dqsk.*t563-dqsf.*(alpha3_1.*t7.*t15.*t22.*1.2e+1-alpha3_2.*t7.*t15.*t22.*2.4e+1+alpha3_3.*t7.*t17.*t19.*1.2e+1+alpha3_3.*t7.*t15.*t22.*1.2e+1-alpha3_4.*t7.*t17.*t19.*2.4e+1+alpha3_5.*t7.*t17.*t19.*1.2e+1-alpha3_2.*t7.*t16.*t18.*t21.*2.4e+1+alpha3_3.*t7.*t16.*t18.*t21.*4.8e+1-alpha3_4.*t7.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t575-dqnsk.*t577-dqsh.*t574-dqsk.*t573-dqsf.*(alpha4_1.*t7.*t15.*t22.*1.2e+1-alpha4_2.*t7.*t15.*t22.*2.4e+1+alpha4_3.*t7.*t17.*t19.*1.2e+1+alpha4_3.*t7.*t15.*t22.*1.2e+1-alpha4_4.*t7.*t17.*t19.*2.4e+1+alpha4_5.*t7.*t17.*t19.*1.2e+1-alpha4_2.*t7.*t16.*t18.*t21.*2.4e+1+alpha4_3.*t7.*t16.*t18.*t21.*4.8e+1-alpha4_4.*t7.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t548-dqnsk.*t549-dqsf.*t543-dqsh.*t546-dqsk.*(alpha1_1.*t8.*t15.*t22.*1.2e+1-alpha1_2.*t8.*t15.*t22.*2.4e+1+alpha1_3.*t8.*t17.*t19.*1.2e+1+alpha1_3.*t8.*t15.*t22.*1.2e+1-alpha1_4.*t8.*t17.*t19.*2.4e+1+alpha1_5.*t8.*t17.*t19.*1.2e+1-alpha1_2.*t8.*t16.*t18.*t21.*2.4e+1+alpha1_3.*t8.*t16.*t18.*t21.*4.8e+1-alpha1_4.*t8.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t558-dqnsk.*t559-dqsf.*t553-dqsh.*t556-dqsk.*(alpha2_1.*t8.*t15.*t22.*1.2e+1-alpha2_2.*t8.*t15.*t22.*2.4e+1+alpha2_3.*t8.*t17.*t19.*1.2e+1+alpha2_3.*t8.*t15.*t22.*1.2e+1-alpha2_4.*t8.*t17.*t19.*2.4e+1+alpha2_5.*t8.*t17.*t19.*1.2e+1-alpha2_2.*t8.*t16.*t18.*t21.*2.4e+1+alpha2_3.*t8.*t16.*t18.*t21.*4.8e+1-alpha2_4.*t8.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t568-dqnsk.*t569-dqsf.*t563-dqsh.*t566-dqsk.*(alpha3_1.*t8.*t15.*t22.*1.2e+1-alpha3_2.*t8.*t15.*t22.*2.4e+1+alpha3_3.*t8.*t17.*t19.*1.2e+1+alpha3_3.*t8.*t15.*t22.*1.2e+1-alpha3_4.*t8.*t17.*t19.*2.4e+1+alpha3_5.*t8.*t17.*t19.*1.2e+1-alpha3_2.*t8.*t16.*t18.*t21.*2.4e+1+alpha3_3.*t8.*t16.*t18.*t21.*4.8e+1-alpha3_4.*t8.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t578-dqnsk.*t579-dqsf.*t573-dqsh.*t576-dqsk.*(alpha4_1.*t8.*t15.*t22.*1.2e+1-alpha4_2.*t8.*t15.*t22.*2.4e+1+alpha4_3.*t8.*t17.*t19.*1.2e+1+alpha4_3.*t8.*t15.*t22.*1.2e+1-alpha4_4.*t8.*t17.*t19.*2.4e+1+alpha4_5.*t8.*t17.*t19.*1.2e+1-alpha4_2.*t8.*t16.*t18.*t21.*2.4e+1+alpha4_3.*t8.*t16.*t18.*t21.*4.8e+1-alpha4_4.*t8.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t550-dqnsk.*t551-dqsf.*t544-dqsk.*t546-dqsh.*(alpha1_1.*t9.*t15.*t22.*1.2e+1-alpha1_2.*t9.*t15.*t22.*2.4e+1+alpha1_3.*t9.*t17.*t19.*1.2e+1+alpha1_3.*t9.*t15.*t22.*1.2e+1-alpha1_4.*t9.*t17.*t19.*2.4e+1+alpha1_5.*t9.*t17.*t19.*1.2e+1-alpha1_2.*t9.*t16.*t18.*t21.*2.4e+1+alpha1_3.*t9.*t16.*t18.*t21.*4.8e+1-alpha1_4.*t9.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t560-dqnsk.*t561-dqsf.*t554-dqsk.*t556-dqsh.*(alpha2_1.*t9.*t15.*t22.*1.2e+1-alpha2_2.*t9.*t15.*t22.*2.4e+1+alpha2_3.*t9.*t17.*t19.*1.2e+1+alpha2_3.*t9.*t15.*t22.*1.2e+1-alpha2_4.*t9.*t17.*t19.*2.4e+1+alpha2_5.*t9.*t17.*t19.*1.2e+1-alpha2_2.*t9.*t16.*t18.*t21.*2.4e+1+alpha2_3.*t9.*t16.*t18.*t21.*4.8e+1-alpha2_4.*t9.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t570-dqnsk.*t571-dqsf.*t564-dqsk.*t566-dqsh.*(alpha3_1.*t9.*t15.*t22.*1.2e+1-alpha3_2.*t9.*t15.*t22.*2.4e+1+alpha3_3.*t9.*t17.*t19.*1.2e+1+alpha3_3.*t9.*t15.*t22.*1.2e+1-alpha3_4.*t9.*t17.*t19.*2.4e+1+alpha3_5.*t9.*t17.*t19.*1.2e+1-alpha3_2.*t9.*t16.*t18.*t21.*2.4e+1+alpha3_3.*t9.*t16.*t18.*t21.*4.8e+1-alpha3_4.*t9.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t580-dqnsk.*t581-dqsf.*t574-dqsk.*t576-dqsh.*(alpha4_1.*t9.*t15.*t22.*1.2e+1-alpha4_2.*t9.*t15.*t22.*2.4e+1+alpha4_3.*t9.*t17.*t19.*1.2e+1+alpha4_3.*t9.*t15.*t22.*1.2e+1-alpha4_4.*t9.*t17.*t19.*2.4e+1+alpha4_5.*t9.*t17.*t19.*1.2e+1-alpha4_2.*t9.*t16.*t18.*t21.*2.4e+1+alpha4_3.*t9.*t16.*t18.*t21.*4.8e+1-alpha4_4.*t9.*t16.*t18.*t21.*2.4e+1),-dqnsk.*t552-dqsf.*t545-dqsh.*t550-dqsk.*t548-dqnsh.*(alpha1_1.*t10.*t15.*t22.*1.2e+1-alpha1_2.*t10.*t15.*t22.*2.4e+1+alpha1_3.*t10.*t17.*t19.*1.2e+1+alpha1_3.*t10.*t15.*t22.*1.2e+1-alpha1_4.*t10.*t17.*t19.*2.4e+1+alpha1_5.*t10.*t17.*t19.*1.2e+1-alpha1_2.*t10.*t16.*t18.*t21.*2.4e+1+alpha1_3.*t10.*t16.*t18.*t21.*4.8e+1-alpha1_4.*t10.*t16.*t18.*t21.*2.4e+1),-dqnsk.*t562-dqsf.*t555-dqsh.*t560-dqsk.*t558-dqnsh.*(alpha2_1.*t10.*t15.*t22.*1.2e+1-alpha2_2.*t10.*t15.*t22.*2.4e+1+alpha2_3.*t10.*t17.*t19.*1.2e+1+alpha2_3.*t10.*t15.*t22.*1.2e+1-alpha2_4.*t10.*t17.*t19.*2.4e+1+alpha2_5.*t10.*t17.*t19.*1.2e+1-alpha2_2.*t10.*t16.*t18.*t21.*2.4e+1+alpha2_3.*t10.*t16.*t18.*t21.*4.8e+1-alpha2_4.*t10.*t16.*t18.*t21.*2.4e+1),-dqnsk.*t572-dqsf.*t565-dqsh.*t570-dqsk.*t568-dqnsh.*(alpha3_1.*t10.*t15.*t22.*1.2e+1-alpha3_2.*t10.*t15.*t22.*2.4e+1+alpha3_3.*t10.*t17.*t19.*1.2e+1+alpha3_3.*t10.*t15.*t22.*1.2e+1-alpha3_4.*t10.*t17.*t19.*2.4e+1+alpha3_5.*t10.*t17.*t19.*1.2e+1-alpha3_2.*t10.*t16.*t18.*t21.*2.4e+1+alpha3_3.*t10.*t16.*t18.*t21.*4.8e+1-alpha3_4.*t10.*t16.*t18.*t21.*2.4e+1),-dqnsk.*t582-dqsf.*t575-dqsh.*t580-dqsk.*t578-dqnsh.*(alpha4_1.*t10.*t15.*t22.*1.2e+1-alpha4_2.*t10.*t15.*t22.*2.4e+1+alpha4_3.*t10.*t17.*t19.*1.2e+1+alpha4_3.*t10.*t15.*t22.*1.2e+1-alpha4_4.*t10.*t17.*t19.*2.4e+1+alpha4_5.*t10.*t17.*t19.*1.2e+1-alpha4_2.*t10.*t16.*t18.*t21.*2.4e+1+alpha4_3.*t10.*t16.*t18.*t21.*4.8e+1-alpha4_4.*t10.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t552-dqsf.*t547-dqsh.*t551-dqsk.*t549-dqnsk.*(alpha1_1.*t11.*t15.*t22.*1.2e+1-alpha1_2.*t11.*t15.*t22.*2.4e+1+alpha1_3.*t11.*t17.*t19.*1.2e+1+alpha1_3.*t11.*t15.*t22.*1.2e+1-alpha1_4.*t11.*t17.*t19.*2.4e+1+alpha1_5.*t11.*t17.*t19.*1.2e+1-alpha1_2.*t11.*t16.*t18.*t21.*2.4e+1+alpha1_3.*t11.*t16.*t18.*t21.*4.8e+1-alpha1_4.*t11.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t562-dqsf.*t557-dqsh.*t561-dqsk.*t559-dqnsk.*(alpha2_1.*t11.*t15.*t22.*1.2e+1-alpha2_2.*t11.*t15.*t22.*2.4e+1+alpha2_3.*t11.*t17.*t19.*1.2e+1+alpha2_3.*t11.*t15.*t22.*1.2e+1-alpha2_4.*t11.*t17.*t19.*2.4e+1+alpha2_5.*t11.*t17.*t19.*1.2e+1-alpha2_2.*t11.*t16.*t18.*t21.*2.4e+1+alpha2_3.*t11.*t16.*t18.*t21.*4.8e+1-alpha2_4.*t11.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t572-dqsf.*t567-dqsh.*t571-dqsk.*t569-dqnsk.*(alpha3_1.*t11.*t15.*t22.*1.2e+1-alpha3_2.*t11.*t15.*t22.*2.4e+1+alpha3_3.*t11.*t17.*t19.*1.2e+1+alpha3_3.*t11.*t15.*t22.*1.2e+1-alpha3_4.*t11.*t17.*t19.*2.4e+1+alpha3_5.*t11.*t17.*t19.*1.2e+1-alpha3_2.*t11.*t16.*t18.*t21.*2.4e+1+alpha3_3.*t11.*t16.*t18.*t21.*4.8e+1-alpha3_4.*t11.*t16.*t18.*t21.*2.4e+1),-dqnsh.*t582-dqsf.*t577-dqsh.*t581-dqsk.*t579-dqnsk.*(alpha4_1.*t11.*t15.*t22.*1.2e+1-alpha4_2.*t11.*t15.*t22.*2.4e+1+alpha4_3.*t11.*t17.*t19.*1.2e+1+alpha4_3.*t11.*t15.*t22.*1.2e+1-alpha4_4.*t11.*t17.*t19.*2.4e+1+alpha4_5.*t11.*t17.*t19.*1.2e+1-alpha4_2.*t11.*t16.*t18.*t21.*2.4e+1+alpha4_3.*t11.*t16.*t18.*t21.*4.8e+1-alpha4_4.*t11.*t16.*t18.*t21.*2.4e+1)],[4,5]);
