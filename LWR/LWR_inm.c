/* (********************************************) */
/* (** SYMORO+ : SYmbolic MOdelling of RObots **) */
/* (**========================================**) */
/* (**      IRCCyN-ECN - 1, rue de la Noe     **) */
/* (**      B.P.92101                         **) */
/* (**      44321 Nantes cedex 3, FRANCE      **) */
/* (**      www.irccyn.ec-nantes.fr           **) */
/* (********************************************) */


/*    Name of file : C:/Users/Guillaume/Desktop/SYMORO_1.6.0/Robots/LWR/LWR.inm */




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

/*                       Inertia matrix */
/* Equations: */

#include <stdio.h>
#include <math.h>

#define sign(x) (x<0.0 ? -1.0 : (x==0.0 ? 0.0 : 1.0))

/* Declaration of global input variables */
extern double q2, q3, q4, q5, q6, q7, MX7, MY7, XX7, XY7;
extern double YY7, XZ7, YZ7, XX6, XY6, XZ6, YY6, ZZ7, YZ6, ZZ6;
extern double MX6, MY6, MZ7, MZ6, M6, M7, XX5, XY5, XZ5, YY5;
extern double YZ5, ZZ5, MX5, MY5, MZ5, M5, R5, XX4, XY4, XZ4;
extern double YY4, YZ4, ZZ4, MX4, MY4, MZ4, M4, XX3, XY3, XZ3;
extern double YY3, YZ3, ZZ3, MX3, MY3, MZ3, M3, R3, XX2, XY2;
extern double XZ2, YY2, YZ2, ZZ2, ZZ1, IA1, IA2, IA3, IA4, IA5;
extern double IA6, IA7;

/* Declaration of global output variables */
extern double A[8][8];

/* Declaration of the function */
void LWR_inm()
{
double S2, C2, S3, C3, S4, C4, S5, C5, S6, C6;
double S7, C7, AS17, AS37, AJ117, AJ127, AJ137, AJ317, AJ327, AJ337;
double AJA117, AJA217, AJA317, AJA337, XXP6, XYP6, XZP6, YYP6, YZP6, ZZP6;
double MXP6, MYP6, MZP6, MP6, AS16, AS36, AJ116, AJ126, AJ136, AJ216;
double AJ226, AJ316, AJ326, AJ336, AJA116, AJA216, AJA316, AJA336, XXP5, XYP5;
double XZP5, YYP5, YZP5, ZZP5, MXP5, MYP5, MZP5, MP5, AS15, AS35;
double AJ115, AJ125, AJ135, AJ215, AJ225, AJ315, AJ325, AJ335, AJA115, AJA215;
double AJA315, AJA335, PAS115, PAS125, PAS325, PAS335, XXP4, XYP4, XZP4, YYP4;
double YZP4, ZZP4, MXP4, MYP4, MZP4, MP4, AS14, AS34, AJ114, AJ124;
double AJ134, AJ214, AJ224, AJ314, AJ324, AJ334, AJA114, AJA214, AJA314, AJA334;
double XXP3, XYP3, XZP3, YYP3, YZP3, ZZP3, MXP3, MYP3, MZP3, MP3;
double AS13, AS33, AJ113, AJ123, AJ133, AJ213, AJ223, AJ313, AJ323, AJ333;
double AJA113, AJA213, AJA313, AJA333, PAS113, PAS123, PAS323, PAS333, XXP2, XYP2;
double XZP2, YYP2, YZP2, ZZP2, AJ312, AJ322, AJ332, AJA332, ZZP1, EC12;
double EC32, NC12, NC32, NC33, ED12, ED13, ED33, ND13, ND33, ND34;
double EE12, EE32, NE12, NE32, EE13, NE13, NE33, EE14, EE34, NE14;
double NE34, NE35, EF12, EF32, EF13, EF33, NF13, NF33, EF14, NF14;
double NF34, EF15, EF35, NF15, NF35, NF36, EG12, EG32, EG13, EG33;
double NG13, NG33, EG14, EG34, NG14, NG34, EG15, NG15, NG35, EG16;
double EG36, NG16, NG36, NG37;

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
	AS17=C7*MX7 - MY7*S7;
	AS37=-(C7*MY7) - MX7*S7;
	AJ117=C7*XX7 - S7*XY7;
	AJ127=C7*XY7 - S7*YY7;
	AJ137=C7*XZ7 - S7*YZ7;
	AJ317=-(S7*XX7) - C7*XY7;
	AJ327=-(S7*XY7) - C7*YY7;
	AJ337=-(S7*XZ7) - C7*YZ7;
	AJA117=AJ117*C7 - AJ127*S7;
	AJA217=C7*XZ7 - S7*YZ7;
	AJA317=AJ317*C7 - AJ327*S7;
	AJA337=-(AJ327*C7) - AJ317*S7;
	XXP6=AJA117 + XX6;
	XYP6=AJA217 + XY6;
	XZP6=AJA317 + XZ6;
	YYP6=YY6 + ZZ7;
	YZP6=AJ337 + YZ6;
	ZZP6=AJA337 + ZZ6;
	MXP6=AS17 + MX6;
	MYP6=MY6 + MZ7;
	MZP6=AS37 + MZ6;
	MP6=M6 + M7;
	AS16=C6*MXP6 - MYP6*S6;
	AS36=C6*MYP6 + MXP6*S6;
	AJ116=C6*XXP6 - S6*(AJA217 + XY6);
	AJ126=C6*XYP6 - S6*YYP6;
	AJ136=C6*XZP6 - S6*YZP6;
	AJ216=-AJA317 - XZ6;
	AJ226=-AJ337 - YZ6;
	AJ316=S6*XXP6 + C6*(AJA217 + XY6);
	AJ326=S6*XYP6 + C6*YYP6;
	AJ336=S6*XZP6 + C6*YZP6;
	AJA116=AJ116*C6 - AJ126*S6;
	AJA216=AJ216*C6 - AJ226*S6;
	AJA316=AJ316*C6 - AJ326*S6;
	AJA336=AJ326*C6 + AJ316*S6;
	XXP5=AJA116 + XX5;
	XYP5=AJA216 + XY5;
	XZP5=AJA316 + XZ5;
	YYP5=YY5 + ZZP6;
	YZP5=-AJ336 + YZ5;
	ZZP5=AJA336 + ZZ5;
	MXP5=AS16 + MX5;
	MYP5=MY5 - MZP6;
	MZP5=AS36 + MZ5;
	MP5=M5 + MP6;
	AS15=C5*MXP5 - MYP5*S5;
	AS35=C5*MYP5 + MXP5*S5;
	AJ115=C5*XXP5 - S5*(AJA216 + XY5);
	AJ125=C5*XYP5 - S5*YYP5;
	AJ135=C5*XZP5 - S5*YZP5;
	AJ215=-AJA316 - XZ5;
	AJ225=AJ336 - YZ5;
	AJ315=S5*XXP5 + C5*(AJA216 + XY5);
	AJ325=S5*XYP5 + C5*YYP5;
	AJ335=S5*XZP5 + C5*YZP5;
	AJA115=AJ115*C5 - AJ125*S5;
	AJA215=AJ215*C5 - AJ225*S5;
	AJA315=AJ315*C5 - AJ325*S5;
	AJA335=AJ325*C5 + AJ315*S5;
	PAS115=-(MZP5*R5);
	PAS125=-(AS15*R5);
	PAS325=-(AS35*R5);
	PAS335=-(MZP5*R5);
	XXP4=AJA115 - 2*PAS115 + MP5*(R5*R5) + XX4;
	XYP4=AJA215 - PAS125 + XY4;
	XZP4=AJA315 + XZ4;
	YYP4=YY4 + ZZP5;
	YZP4=-AJ335 - PAS325 + YZ4;
	ZZP4=AJA335 - 2*PAS335 + MP5*(R5*R5) + ZZ4;
	MXP4=AS15 + MX4;
	MYP4=MY4 - MZP5 - MP5*R5;
	MZP4=AS35 + MZ4;
	MP4=M4 + MP5;
	AS14=C4*MXP4 - MYP4*S4;
	AS34=-(C4*MYP4) - MXP4*S4;
	AJ114=C4*XXP4 - S4*(AJA215 - PAS125 + XY4);
	AJ124=C4*XYP4 - S4*YYP4;
	AJ134=C4*XZP4 - S4*YZP4;
	AJ214=AJA315 + XZ4;
	AJ224=-AJ335 - PAS325 + YZ4;
	AJ314=-(S4*XXP4) - C4*(AJA215 - PAS125 + XY4);
	AJ324=-(S4*XYP4) - C4*YYP4;
	AJ334=-(S4*XZP4) - C4*YZP4;
	AJA114=AJ114*C4 - AJ124*S4;
	AJA214=AJ214*C4 - AJ224*S4;
	AJA314=AJ314*C4 - AJ324*S4;
	AJA334=-(AJ324*C4) - AJ314*S4;
	XXP3=AJA114 + XX3;
	XYP3=AJA214 + XY3;
	XZP3=AJA314 + XZ3;
	YYP3=YY3 + ZZP4;
	YZP3=AJ334 + YZ3;
	ZZP3=AJA334 + ZZ3;
	MXP3=AS14 + MX3;
	MYP3=MY3 + MZP4;
	MZP3=AS34 + MZ3;
	MP3=M3 + MP4;
	AS13=C3*MXP3 - MYP3*S3;
	AS33=-(C3*MYP3) - MXP3*S3;
	AJ113=C3*XXP3 - S3*(AJA214 + XY3);
	AJ123=C3*XYP3 - S3*YYP3;
	AJ133=C3*XZP3 - S3*YZP3;
	AJ213=AJA314 + XZ3;
	AJ223=AJ334 + YZ3;
	AJ313=-(S3*XXP3) - C3*(AJA214 + XY3);
	AJ323=-(S3*XYP3) - C3*YYP3;
	AJ333=-(S3*XZP3) - C3*YZP3;
	AJA113=AJ113*C3 - AJ123*S3;
	AJA213=AJ213*C3 - AJ223*S3;
	AJA313=AJ313*C3 - AJ323*S3;
	AJA333=-(AJ323*C3) - AJ313*S3;
	PAS113=-(MZP3*R3);
	PAS123=AS13*R3;
	PAS323=AS33*R3;
	PAS333=-(MZP3*R3);
	XXP2=AJA113 - 2*PAS113 + MP3*(R3*R3) + XX2;
	XYP2=AJA213 - PAS123 + XY2;
	XZP2=AJA313 + XZ2;
	YYP2=YY2 + ZZP3;
	YZP2=AJ333 - PAS323 + YZ2;
	ZZP2=AJA333 - 2*PAS333 + MP3*(R3*R3) + ZZ2;
	AJ312=S2*XXP2 + C2*(AJA213 - PAS123 + XY2);
	AJ322=S2*XYP2 + C2*YYP2;
	AJ332=S2*XZP2 + C2*YZP2;
	AJA332=AJ322*C2 + AJ312*S2;
	ZZP1=AJA332 + ZZ1;
	EC12=-(C3*MYP3) - MXP3*S3;
	EC32=-(C3*MXP3) + MYP3*S3;
	NC12=AJ133 + EC32*R3;
	NC32=AJ333 - EC12*R3;
	NC33=NC12*S2 + C2*ZZP3;
	ED12=-(C4*MYP4) - MXP4*S4;
	ED13=C3*ED12;
	ED33=-(ED12*S3);
	ND13=AJ134*C3 + ED33*R3 - S3*ZZP4;
	ND33=-(ED13*R3) - AJ134*S3 - C3*ZZP4;
	ND34=AJ334*C2 + ND13*S2;
	EE12=-(C5*MYP5) - MXP5*S5;
	EE32=C5*MXP5 - MYP5*S5;
	NE12=AJ135 - EE32*R5;
	NE32=AJ335 + EE12*R5;
	EE13=C4*EE12;
	NE13=C4*NE12 + S4*ZZP5;
	NE33=-(NE12*S4) + C4*ZZP5;
	EE14=C3*EE13 - EE32*S3;
	EE34=-(C3*EE32) - EE13*S3;
	NE14=C3*NE13 + EE34*R3 - NE32*S3;
	NE34=-(C3*NE32) - EE14*R3 - NE13*S3;
	NE35=C2*NE33 + NE14*S2;
	EF12=-(C6*MYP6) - MXP6*S6;
	EF32=C6*MXP6 - MYP6*S6;
	EF13=C5*EF12;
	EF33=EF12*S5;
	NF13=AJ136*C5 - EF33*R5 + S5*ZZP6;
	NF33=EF13*R5 + AJ136*S5 - C5*ZZP6;
	EF14=C4*EF13 + EF32*S4;
	NF14=C4*NF13 + AJ336*S4;
	NF34=AJ336*C4 - NF13*S4;
	EF15=C3*EF14 - EF33*S3;
	EF35=-(C3*EF33) - EF14*S3;
	NF15=C3*NF14 + EF35*R3 - NF33*S3;
	NF35=-(C3*NF33) - EF15*R3 - NF14*S3;
	NF36=C2*NF34 + NF15*S2;
	EG12=-(C7*MY7) - MX7*S7;
	EG32=-(C7*MX7) + MY7*S7;
	EG13=C6*EG12;
	EG33=EG12*S6;
	NG13=AJ137*C6 - S6*ZZ7;
	NG33=AJ137*S6 + C6*ZZ7;
	EG14=C5*EG13 + EG32*S5;
	EG34=-(C5*EG32) + EG13*S5;
	NG14=C5*NG13 - EG34*R5 + AJ337*S5;
	NG34=-(AJ337*C5) + EG14*R5 + NG13*S5;
	EG15=C4*EG14 + EG33*S4;
	NG15=C4*NG14 + NG33*S4;
	NG35=C4*NG33 - NG14*S4;
	EG16=C3*EG15 - EG34*S3;
	EG36=-(C3*EG34) - EG15*S3;
	NG16=C3*NG15 + EG36*R3 - NG34*S3;
	NG36=-(C3*NG34) - EG16*R3 - NG15*S3;
	NG37=C2*NG35 + NG16*S2;
	A[1][1]=IA1 + ZZP1;
	A[2][1]=AJ332;
	A[3][1]=NC33;
	A[4][1]=ND34;
	A[5][1]=NE35;
	A[6][1]=NF36;
	A[7][1]=NG37;
	A[2][2]=IA2 + ZZP2;
	A[3][2]=NC32;
	A[4][2]=ND33;
	A[5][2]=NE34;
	A[6][2]=NF35;
	A[7][2]=NG36;
	A[3][3]=IA3 + ZZP3;
	A[4][3]=AJ334;
	A[5][3]=NE33;
	A[6][3]=NF34;
	A[7][3]=NG35;
	A[4][4]=IA4 + ZZP4;
	A[5][4]=NE32;
	A[6][4]=NF33;
	A[7][4]=NG34;
	A[5][5]=IA5 + ZZP5;
	A[6][5]=AJ336;
	A[7][5]=NG33;
	A[6][6]=IA6 + ZZP6;
	A[7][6]=AJ337;
	A[7][7]=IA7 + ZZ7;
}

/* *=* */
/* Number of operations : 212 '+' or '-', 260 '*' or '/' */

/*  QDP=  */
/* {qdd1, qdd2, qdd3, qdd4, qdd6, qdd7} */

