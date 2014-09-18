/* (********************************************) */
/* (** SYMORO+ : SYmbolic MOdelling of RObots **) */
/* (**========================================**) */
/* (**      IRCCyN-ECN - 1, rue de la Noe     **) */
/* (**      B.P.92101                         **) */
/* (**      44321 Nantes cedex 3, FRANCE      **) */
/* (**      www.irccyn.ec-nantes.fr           **) */
/* (********************************************) */


/*    Name of file : C:/Users/Guillaume/Desktop/SYMORO_1.6.0/Robots/LWR/LWR.ccg */




/*      Geometric parameters    */


/* j     ant   mu    sigma gamma b     alpha d     theta r */


/* 1     0     1     0     0     0     0     0     q1    0 */

/*                                     Pi */
/* 2     1     1     0     0     0     --    0     q2    0 */
/*                                     2 */
/*                                     -Pi */
/* 3     2     1     0     0     0     ---   0     q3    R3 */
/*                                      2 */
/*                                     -Pi */
/* 4     3     1     0     0     0     ---   0     q4    0 */
/*                                      2 */
/*                                     Pi */
/* 5     4     1     0     0     0     --    0     q5    R5 */
/*                                     2 */
/*                                     Pi */
/* 6     5     1     0     0     0     --    0     q6    0 */
/*                                     2 */
/*                                     -Pi */
/* 7     6     1     0     0     0     ---   0     q7    0 */
/*                                      2 */



/*              Inertial parameters */

/* j     XX    XY    XZ    YY    YZ    ZZ    MX    MY    MZ    M     Ia */

/* 1     XX1   XY1   XZ1   YY1   YZ1   ZZ1   MX1   MY1   MZ1   M1    IA1 */

/* 2     XX2   XY2   XZ2   YY2   YZ2   ZZ2   MX2   MY2   MZ2   M2    IA2 */

/* 3     XX3   XY3   XZ3   YY3   YZ3   ZZ3   MX3   MY3   MZ3   M3    IA3 */

/* 4     XX4   XY4   XZ4   YY4   YZ4   ZZ4   MX4   MY4   MZ4   M4    IA4 */

/* 5     XX5   XY5   XZ5   YY5   YZ5   ZZ5   MX5   MY5   MZ5   M5    IA5 */

/* 6     XX6   XY6   XZ6   YY6   YZ6   ZZ6   MX6   MY6   MZ6   M6    IA6 */

/* 7     XX7   XY7   XZ7   YY7   YZ7   ZZ7   MX7   MY7   MZ7   M7    IA7 */



/*  External forces,friction parameters, joint velocities and accelerations */

/* j      FX     FY     FZ     CX     CY     CZ     FS     FV     QP     QDP */

/* 1      0      0      0      0      0      0      FS1    FV1    qd1    qdd1 */

/* 2      0      0      0      0      0      0      FS2    FV2    qd2    qdd2 */

/* 3      0      0      0      0      0      0      FS3    FV3    qd3    qdd3 */

/* 4      0      0      0      0      0      0      FS4    FV4    qd4    qdd4 */

/* 5      0      0      0      0      0      0      FS5    FV5    qd4    qdd4 */

/* 6      0      0      0      0      0      0      FS6    FV6    qd6    qdd6 */

/* 7      FX7    FY7    FZ7    CX7    CY7    CZ7    FS7    FV7    qd7    qdd7 */

/* Base velocity, base accelerations, and gravity */

/* j     W0    WP0   V0    VP0   G */

/* 1     0     0     0     0     GX */

/* 2     0     0     0     0     GY */

/* 3     0     0     0     0     GZ */

/*   Centrifugal, Coriolis and gravity Torque */

/* Equations: */

#include <stdio.h>
#include <math.h>

#define sign(x) (x<0.0 ? -1.0 : (x==0.0 ? 0.0 : 1.0))

/* Declaration of global input variables */
extern double q1, q2, q3, q4, q5, q6, q7, GX, GY, qd1;
extern double qd2, GZ, YY2, ZZ2, XX2, XY2, XZ2, YZ2, qd3, R3;
extern double MX3, MY3, MZ3, M3, YY3, ZZ3, XX3, XY3, XZ3, YZ3;
extern double qd4, MX4, MY4, MZ4, M4, YY4, ZZ4, XX4, XY4, XZ4;
extern double YZ4, R5, MX5, MY5, MZ5, M5, YY5, ZZ5, XX5, XY5;
extern double XZ5, YZ5, qd6, MX6, MY6, MZ6, M6, YY6, ZZ6, XX6;
extern double XY6, XZ6, YZ6, qd7, MX7, MY7, MZ7, M7, YY7, ZZ7;
extern double XX7, XY7, XZ7, YZ7, FX7, FY7, FZ7, CX7, CY7, CZ7;
extern double MY2, MZ2, MX2, MY1, MX1, FV1, FS1, FV2, FS2, FV3;
extern double FS3, FV4, FS4, FV5, FS5, FV6, FS6, FV7, FS7;

/* Declaration of global output variables */
extern double H1, H2, H3, H4, H5, H6, H7;

/* Declaration of the function */
void LWR_ccg()
{
double S1, C1, S2, C2, S3, C3, S4, C4, S5, C5;
double S6, C6, S7, C7, VP11, VP21, WI12, WI22, WP12, WP22;
double DV112, DV222, DV332, DV122, DV132, DV232, U132, U222, U232, U312;
double U322, VP12, VP22, PIS12, PIS22, PIS32, No12, No22, No32, WI13;
double WI23, W33, WP13, WP23, DV113, DV223, DV333, DV123, DV133, DV233;
double U113, U123, U133, U213, U223, U233, U313, U323, VSP13, VSP23;
double VSP33, VP13, VP23, F13, F23, PIS13, PIS23, PIS33, No13, No23;
double No33, WI14, WI24, W34, WP14, WP24, DV114, DV224, DV334, DV124;
double DV134, DV234, U114, U124, U134, U214, U224, U234, U314, U324;
double U334, VP14, VP24, F14, F24, F34, PIS14, PIS24, PIS34, No14;
double No24, No34, WI15, WI25, W35, WP15, WP25, DV115, DV225, DV335;
double DV125, DV135, DV235, U115, U125, U135, U215, U225, U235, U315;
double U325, U335, VSP15, VSP25, VSP35, VP15, VP25, F15, F25, F35;
double PIS15, PIS25, PIS35, No15, No25, No35, WI16, WI26, W36, WP16;
double WP26, DV116, DV226, DV336, DV126, DV136, DV236, U116, U126, U136;
double U216, U226, U236, U316, U326, U336, VP16, VP26, F16, F26;
double F36, PIS16, PIS26, PIS36, No16, No26, No36, WI17, WI27, W37;
double WP17, WP27, DV117, DV227, DV337, DV127, DV137, DV237, U117, U127;
double U137, U217, U227, U237, U317, U327, U337, VP17, VP27, F17;
double F27, F37, PIS17, PIS27, PIS37, No17, No27, No37, E17, E27;
double E37, N17, N27, N37, FDI17, FDI37, E16, E26, E36, N16;
double N26, N36, FDI16, FDI36, E15, E25, E35, N15, N25, N35;
double FDI15, FDI35, E14, E24, E34, N14, N24, N34, FDI14, E13;
double E23, N13, N23, N33, FDI13, FDI33, N12, N22, N32, N31;

	S1=sin(q1);
	C1=cos(q1);
	S2=sin(q2);
	C2=cos(q2);
	S3=sin(q3);
	C3=cos(q3);
	S4=sin(q4);
	C4=cos(q4);
	S5=sin(q5);
	C5=cos(q5);
	S6=sin(q6);
	C6=cos(q6);
	S7=sin(q7);
	C7=cos(q7);
	VP11=-(C1*GX) - GY*S1;
	VP21=-(C1*GY) + GX*S1;
	WI12=qd1*S2;
	WI22=C2*qd1;
	WP12=qd2*WI22;
	WP22=-(qd2*WI12);
	DV112=-(WI12*WI12);
	DV222=-(WI22*WI22);
	DV332=-(qd2*qd2);
	DV122=WI12*WI22;
	DV132=qd2*WI12;
	DV232=qd2*WI22;
	U132=DV132 + WP22;
	U222=DV112 + DV332;
	U232=DV232 - WP12;
	U312=DV132 - WP22;
	U322=DV232 + WP12;
	VP12=-(GZ*S2) + C2*VP11;
	VP22=-(C2*GZ) - S2*VP11;
	PIS12=-YY2 + ZZ2;
	PIS22=XX2 - ZZ2;
	PIS32=-XX2 + YY2;
	No12=DV232*PIS12 + WP12*XX2 - U312*XY2 + DV122*XZ2 + (-DV222 + DV332)*YZ2;
	No22=DV132*PIS22 + U322*XY2 + (DV112 - DV332)*XZ2 + WP22*YY2 - DV122*YZ2;
	No32=DV122*PIS32 + (-DV112 + DV222)*XY2 - U232*XZ2 + U132*YZ2;
	WI13=-(qd2*S3) + C3*WI12;
	WI23=-(C3*qd2) - S3*WI12;
	W33=qd3 + WI22;
	WP13=qd3*WI23 + C3*WP12;
	WP23=-(qd3*WI13) - S3*WP12;
	DV113=-(WI13*WI13);
	DV223=-(WI23*WI23);
	DV333=-(W33*W33);
	DV123=WI13*WI23;
	DV133=W33*WI13;
	DV233=W33*WI23;
	U113=DV223 + DV333;
	U123=DV123 - WP22;
	U133=DV133 + WP23;
	U213=DV123 + WP22;
	U223=DV113 + DV333;
	U233=DV233 - WP13;
	U313=DV133 - WP23;
	U323=DV233 + WP13;
	VSP13=DV122*R3 + VP12;
	VSP23=R3*U222 + VP22;
	VSP33=R3*U322 - VP21;
	VP13=C3*VSP13 - S3*VSP33;
	VP23=-(S3*VSP13) - C3*VSP33;
	F13=MX3*U113 + MY3*U123 + MZ3*U133 + M3*VP13;
	F23=MX3*U213 + MY3*U223 + MZ3*U233 + M3*VP23;
	PIS13=-YY3 + ZZ3;
	PIS23=XX3 - ZZ3;
	PIS33=-XX3 + YY3;
	No13=DV233*PIS13 + WP13*XX3 - U313*XY3 + U213*XZ3 + (-DV223 + DV333)*YZ3;
	No23=DV133*PIS23 + U323*XY3 + (DV113 - DV333)*XZ3 + WP23*YY3 - U123*YZ3;
	No33=DV123*PIS33 + (-DV113 + DV223)*XY3 - U233*XZ3 + U133*YZ3 + WP22*ZZ3;
	WI14=-(S4*W33) + C4*WI13;
	WI24=-(C4*W33) - S4*WI13;
	W34=qd4 + WI23;
	WP14=qd4*WI24 + C4*WP13 - S4*WP22;
	WP24=-(qd4*WI14) - S4*WP13 - C4*WP22;
	DV114=-(WI14*WI14);
	DV224=-(WI24*WI24);
	DV334=-(W34*W34);
	DV124=WI14*WI24;
	DV134=W34*WI14;
	DV234=W34*WI24;
	U114=DV224 + DV334;
	U124=DV124 - WP23;
	U134=DV134 + WP24;
	U214=DV124 + WP23;
	U224=DV114 + DV334;
	U234=DV234 - WP14;
	U314=DV134 - WP24;
	U324=DV234 + WP14;
	U334=DV114 + DV224;
	VP14=C4*VP13 - S4*VSP23;
	VP24=-(S4*VP13) - C4*VSP23;
	F14=MX4*U114 + MY4*U124 + MZ4*U134 + M4*VP14;
	F24=MX4*U214 + MY4*U224 + MZ4*U234 + M4*VP24;
	F34=MX4*U314 + MY4*U324 + MZ4*U334 + M4*VP23;
	PIS14=-YY4 + ZZ4;
	PIS24=XX4 - ZZ4;
	PIS34=-XX4 + YY4;
	No14=DV234*PIS14 + WP14*XX4 - U314*XY4 + U214*XZ4 + (-DV224 + DV334)*YZ4;
	No24=DV134*PIS24 + U324*XY4 + (DV114 - DV334)*XZ4 + WP24*YY4 - U124*YZ4;
	No34=DV124*PIS34 + (-DV114 + DV224)*XY4 - U234*XZ4 + U134*YZ4 + WP23*ZZ4;
	WI15=S5*W34 + C5*WI14;
	WI25=C5*W34 - S5*WI14;
	W35=qd4 - WI24;
	WP15=qd4*WI25 + C5*WP14 + S5*WP23;
	WP25=-(qd4*WI15) - S5*WP14 + C5*WP23;
	DV115=-(WI15*WI15);
	DV225=-(WI25*WI25);
	DV335=-(W35*W35);
	DV125=WI15*WI25;
	DV135=W35*WI15;
	DV235=W35*WI25;
	U115=DV225 + DV335;
	U125=DV125 + WP24;
	U135=DV135 + WP25;
	U215=DV125 - WP24;
	U225=DV115 + DV335;
	U235=DV235 - WP15;
	U315=DV135 - WP25;
	U325=DV235 + WP15;
	U335=DV115 + DV225;
	VSP15=-(R5*U124) + VP14;
	VSP25=-(R5*U224) + VP24;
	VSP35=-(R5*U324) + VP23;
	VP15=C5*VSP15 + S5*VSP35;
	VP25=-(S5*VSP15) + C5*VSP35;
	F15=MX5*U115 + MY5*U125 + MZ5*U135 + M5*VP15;
	F25=MX5*U215 + MY5*U225 + MZ5*U235 + M5*VP25;
	F35=MX5*U315 + MY5*U325 + MZ5*U335 - M5*VSP25;
	PIS15=-YY5 + ZZ5;
	PIS25=XX5 - ZZ5;
	PIS35=-XX5 + YY5;
	No15=DV235*PIS15 + WP15*XX5 - U315*XY5 + U215*XZ5 + (-DV225 + DV335)*YZ5;
	No25=DV135*PIS25 + U325*XY5 + (DV115 - DV335)*XZ5 + WP25*YY5 - U125*YZ5;
	No35=DV125*PIS35 + (-DV115 + DV225)*XY5 - U235*XZ5 + U135*YZ5 - WP24*ZZ5;
	WI16=S6*W35 + C6*WI15;
	WI26=C6*W35 - S6*WI15;
	W36=qd6 - WI25;
	WP16=qd6*WI26 + C6*WP15 - S6*WP24;
	WP26=-(qd6*WI16) - S6*WP15 - C6*WP24;
	DV116=-(WI16*WI16);
	DV226=-(WI26*WI26);
	DV336=-(W36*W36);
	DV126=WI16*WI26;
	DV136=W36*WI16;
	DV236=W36*WI26;
	U116=DV226 + DV336;
	U126=DV126 + WP25;
	U136=DV136 + WP26;
	U216=DV126 - WP25;
	U226=DV116 + DV336;
	U236=DV236 - WP16;
	U316=DV136 - WP26;
	U326=DV236 + WP16;
	U336=DV116 + DV226;
	VP16=C6*VP15 - S6*VSP25;
	VP26=-(S6*VP15) - C6*VSP25;
	F16=MX6*U116 + MY6*U126 + MZ6*U136 + M6*VP16;
	F26=MX6*U216 + MY6*U226 + MZ6*U236 + M6*VP26;
	F36=MX6*U316 + MY6*U326 + MZ6*U336 - M6*VP25;
	PIS16=-YY6 + ZZ6;
	PIS26=XX6 - ZZ6;
	PIS36=-XX6 + YY6;
	No16=DV236*PIS16 + WP16*XX6 - U316*XY6 + U216*XZ6 + (-DV226 + DV336)*YZ6;
	No26=DV136*PIS26 + U326*XY6 + (DV116 - DV336)*XZ6 + WP26*YY6 - U126*YZ6;
	No36=DV126*PIS36 + (-DV116 + DV226)*XY6 - U236*XZ6 + U136*YZ6 - WP25*ZZ6;
	WI17=-(S7*W36) + C7*WI16;
	WI27=-(C7*W36) - S7*WI16;
	W37=qd7 + WI26;
	WP17=qd7*WI27 + C7*WP16 + S7*WP25;
	WP27=-(qd7*WI17) - S7*WP16 + C7*WP25;
	DV117=-(WI17*WI17);
	DV227=-(WI27*WI27);
	DV337=-(W37*W37);
	DV127=WI17*WI27;
	DV137=W37*WI17;
	DV237=W37*WI27;
	U117=DV227 + DV337;
	U127=DV127 - WP26;
	U137=DV137 + WP27;
	U217=DV127 + WP26;
	U227=DV117 + DV337;
	U237=DV237 - WP17;
	U317=DV137 - WP27;
	U327=DV237 + WP17;
	U337=DV117 + DV227;
	VP17=C7*VP16 + S7*VP25;
	VP27=-(S7*VP16) + C7*VP25;
	F17=MX7*U117 + MY7*U127 + MZ7*U137 + M7*VP17;
	F27=MX7*U217 + MY7*U227 + MZ7*U237 + M7*VP27;
	F37=MX7*U317 + MY7*U327 + MZ7*U337 + M7*VP26;
	PIS17=-YY7 + ZZ7;
	PIS27=XX7 - ZZ7;
	PIS37=-XX7 + YY7;
	No17=DV237*PIS17 + WP17*XX7 - U317*XY7 + U217*XZ7 + (-DV227 + DV337)*YZ7;
	No27=DV137*PIS27 + U327*XY7 + (DV117 - DV337)*XZ7 + WP27*YY7 - U127*YZ7;
	No37=DV127*PIS37 + (-DV117 + DV227)*XY7 - U237*XZ7 + U137*YZ7 + WP26*ZZ7;
	E17=F17 + FX7;
	E27=F27 + FY7;
	E37=F37 + FZ7;
	N17=CX7 + No17 + MY7*VP26 - MZ7*VP27;
	N27=CY7 + No27 + MZ7*VP17 - MX7*VP26;
	N37=CZ7 + No37 - MY7*VP17 + MX7*VP27;
	FDI17=C7*E17 - E27*S7;
	FDI37=-(C7*E27) - E17*S7;
	E16=F16 + FDI17;
	E26=E37 + F26;
	E36=F36 + FDI37;
	N16=C7*N17 + No16 - N27*S7 - MY6*VP25 - MZ6*VP26;
	N26=N37 + No26 + MZ6*VP16 + MX6*VP25;
	N36=-(C7*N27) + No36 - N17*S7 - MY6*VP16 + MX6*VP26;
	FDI16=C6*E16 - E26*S6;
	FDI36=C6*E26 + E16*S6;
	E15=F15 + FDI16;
	E25=-E36 + F25;
	E35=F35 + FDI36;
	N15=C6*N16 + No15 - N26*S6 - MZ5*VP25 - MY5*VSP25;
	N25=-N36 + No25 + MZ5*VP15 + MX5*VSP25;
	N35=C6*N26 + No35 + N16*S6 - MY5*VP15 + MX5*VP25;
	FDI15=C5*E15 - E25*S5;
	FDI35=C5*E25 + E15*S5;
	E14=F14 + FDI15;
	E24=-E35 + F24;
	E34=F34 + FDI35;
	N14=C5*N15 + No14 - FDI35*R5 - N25*S5 + MY4*VP23 - MZ4*VP24;
	N24=-N35 + No24 + MZ4*VP14 - MX4*VP23;
	N34=C5*N25 + No34 + FDI15*R5 + N15*S5 - MY4*VP14 + MX4*VP24;
	FDI14=C4*E14 - E24*S4;
	E13=F13 + FDI14;
	E23=E34 + F23;
	N13=C4*N14 + No13 - N24*S4 - MZ3*VP23 + MY3*VSP23;
	N23=N34 + No23 + MZ3*VP13 - MX3*VSP23;
	N33=-(C4*N24) + No33 - N14*S4 - MY3*VP13 + MX3*VP23;
	FDI13=C3*E13 - E23*S3;
	FDI33=-(C3*E23) - E13*S3;
	N12=C3*N13 + No12 + FDI33*R3 - N23*S3 - MY2*VP21 - MZ2*VP22;
	N22=N33 + No22 + MZ2*VP12 + MX2*VP21;
	N32=-(C3*N23) + No32 - FDI13*R3 - N13*S3 - MY2*VP12 + MX2*VP22;
	N31=C2*N22 + N12*S2 - MY1*VP11 + MX1*VP21;
	H1=N31 + FV1*qd1 + FS1*sign(qd1);
	H2=N32 + FV2*qd2 + FS2*sign(qd2);
	H3=N33 + FV3*qd3 + FS3*sign(qd3);
	H4=N34 + FV4*qd4 + FS4*sign(qd4);
	H5=N35 + FV5*qd4 + FS5*sign(qd4);
	H6=N36 + FV6*qd6 + FS6*sign(qd6);
	H7=N37 + FV7*qd7 + FS7*sign(qd7);
}

/* *=* */
/* Number of operations : 359 '+' or '-', 363 '*' or '/' */
