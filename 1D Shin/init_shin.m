% init_shin

shin.I1 = 0.66;
shin.I2 = 0.33;
shin.m1 = 2.25;
shin.m2 = 15;
shin.l1max= 0.3;

shin.spring.Ksp = 2e3;
shin.spring.Ksp2 = 5e5;
shin.spring.kmax = 0.28;
shin.spring.Kb = 1e3;
shin.spring.k0 = 0.25;

shin.damper.Kd = 0.5e2;
shin.damper.Kdmax = 1.5e2;
shin.damper.Kdmin = 0.5e1;
shin.damper.Kd_q = 0;
shin.l1min = shin.l1max - shin.spring.k0;
shin.g = 9.81;


