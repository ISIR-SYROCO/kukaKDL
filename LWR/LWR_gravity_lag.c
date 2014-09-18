/* (********************************************) */
/* (** SYMORO+ : SYmbolic MOdelling of RObots **) */
/* (**========================================**) */
/* (**      IRCCyN-ECN - 1, rue de la Noe     **) */
/* (**      B.P.92101                         **) */
/* (**      44321 Nantes cedex 3, FRANCE      **) */
/* (**      www.irccyn.ec-nantes.fr           **) */
/* (********************************************) */


/*    Name of file : C:/Users/Guillaume/Desktop/SYMORO_1.6.0/Robots/LWR/LWR_gravity.lag */




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




/*   The matrices A, B, C & Q of the  dynamic model (Lagrange method)  */
/* Equations: */

#include <stdio.h>
#include <math.h>

#define sign(x) (x<0.0 ? -1.0 : (x==0.0 ? 0.0 : 1.0))

/* Declaration of global input variables */
extern double q1, q2, q3, q4, q5, q6, q7, MX7, MY7, MX6;
extern double MY6, MZ7, MZ6, M6, M7, MX5, MY5, MZ5, M5, MX4;
extern double MY4, R5, MZ4, M4, MX3, MY3, MZ3, M3, MX2, MY2;
extern double R3, MZ2, MX1, MY1, GX, GY, GZ;

/* Declaration of global output variables */
extern double Q[8];

/* Declaration of the function */
void LWR_gravity_lag()
{
double S1, C1, S2, C2, S3, C3, S4, C4, S5, C5;
double S6, C6, S7, C7, AS17, AS37, MXP6, MYP6, MZP6, MP6;
double AS16, AS36, MXP5, MYP5, MZP5, MP5, AS15, AS35, MXP4, MYP4;
double MZP4, MP4, AS14, AS34, MXP3, MYP3, MZP3, MP3, AS13, AS33;
double MXP2, MYP2, MZP2, AS12, MXP1, MYP1, PMA0112, PMA0122, PMA0212, PMA0222;
double PMA0113, PMA0123, PMA0213, PMA0223, PMA0313, PMA0323, PMA0114, PMA0124, PMA0214, PMA0224;
double PMA0314, PMA0324, PMA0115, PMA0125, PMA0215, PMA0225, PMA0315, PMA0325, PMA0116, PMA0126;
double PMA0216, PMA0226, PMA0316, PMA0326, PMA0117, PMA0127, PMA0217, PMA0227, PMA0317, PMA0327;
double VQ11, VQ21, VQ12, VQ22, VQ32, VQ13, VQ23, VQ33, VQ14, VQ24;
double VQ34, VQ15, VQ25, VQ35, VQ16, VQ26, VQ36, VQ17, VQ27, VQ37;

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
	AS17=C7*MX7 - MY7*S7;
	AS37=-(C7*MY7) - MX7*S7;
	MXP6=AS17 + MX6;
	MYP6=MY6 + MZ7;
	MZP6=AS37 + MZ6;
	MP6=M6 + M7;
	AS16=C6*MXP6 - MYP6*S6;
	AS36=C6*MYP6 + MXP6*S6;
	MXP5=AS16 + MX5;
	MYP5=MY5 - MZP6;
	MZP5=AS36 + MZ5;
	MP5=M5 + MP6;
	AS15=C5*MXP5 - MYP5*S5;
	AS35=C5*MYP5 + MXP5*S5;
	MXP4=AS15 + MX4;
	MYP4=MY4 - MZP5 - MP5*R5;
	MZP4=AS35 + MZ4;
	MP4=M4 + MP5;
	AS14=C4*MXP4 - MYP4*S4;
	AS34=-(C4*MYP4) - MXP4*S4;
	MXP3=AS14 + MX3;
	MYP3=MY3 + MZP4;
	MZP3=AS34 + MZ3;
	MP3=M3 + MP4;
	AS13=C3*MXP3 - MYP3*S3;
	AS33=-(C3*MYP3) - MXP3*S3;
	MXP2=AS13 + MX2;
	MYP2=MY2 + MZP3 + MP3*R3;
	MZP2=AS33 + MZ2;
	AS12=C2*MXP2 - MYP2*S2;
	MXP1=AS12 + MX1;
	MYP1=MY1 - MZP2;
	PMA0112=C1*C2;
	PMA0122=-(C1*S2);
	PMA0212=C2*S1;
	PMA0222=-(S1*S2);
	PMA0113=C3*PMA0112 - S1*S3;
	PMA0123=-(C3*S1) - PMA0112*S3;
	PMA0213=C3*PMA0212 + C1*S3;
	PMA0223=C1*C3 - PMA0212*S3;
	PMA0313=C3*S2;
	PMA0323=-(S2*S3);
	PMA0114=C4*PMA0113 - PMA0122*S4;
	PMA0124=-(C4*PMA0122) - PMA0113*S4;
	PMA0214=C4*PMA0213 - PMA0222*S4;
	PMA0224=-(C4*PMA0222) - PMA0213*S4;
	PMA0314=C4*PMA0313 - C2*S4;
	PMA0324=-(C2*C4) - PMA0313*S4;
	PMA0115=C5*PMA0114 + PMA0123*S5;
	PMA0125=C5*PMA0123 - PMA0114*S5;
	PMA0215=C5*PMA0214 + PMA0223*S5;
	PMA0225=C5*PMA0223 - PMA0214*S5;
	PMA0315=C5*PMA0314 + PMA0323*S5;
	PMA0325=C5*PMA0323 - PMA0314*S5;
	PMA0116=C6*PMA0115 - PMA0124*S6;
	PMA0126=-(C6*PMA0124) - PMA0115*S6;
	PMA0216=C6*PMA0215 - PMA0224*S6;
	PMA0226=-(C6*PMA0224) - PMA0215*S6;
	PMA0316=C6*PMA0315 - PMA0324*S6;
	PMA0326=-(C6*PMA0324) - PMA0315*S6;
	PMA0117=C7*PMA0116 + PMA0125*S7;
	PMA0127=C7*PMA0125 - PMA0116*S7;
	PMA0217=C7*PMA0216 + PMA0225*S7;
	PMA0227=C7*PMA0225 - PMA0216*S7;
	PMA0317=C7*PMA0316 + PMA0325*S7;
	PMA0327=C7*PMA0325 - PMA0316*S7;
	VQ11=-(C1*MYP1) - MXP1*S1;
	VQ21=C1*MXP1 - MYP1*S1;
	VQ12=-(MYP2*PMA0112) + MXP2*PMA0122;
	VQ22=-(MYP2*PMA0212) + MXP2*PMA0222;
	VQ32=C2*MXP2 - MYP2*S2;
	VQ13=-(MYP3*PMA0113) + MXP3*PMA0123;
	VQ23=-(MYP3*PMA0213) + MXP3*PMA0223;
	VQ33=-(MYP3*PMA0313) + MXP3*PMA0323;
	VQ14=-(MYP4*PMA0114) + MXP4*PMA0124;
	VQ24=-(MYP4*PMA0214) + MXP4*PMA0224;
	VQ34=-(MYP4*PMA0314) + MXP4*PMA0324;
	VQ15=-(MYP5*PMA0115) + MXP5*PMA0125;
	VQ25=-(MYP5*PMA0215) + MXP5*PMA0225;
	VQ35=-(MYP5*PMA0315) + MXP5*PMA0325;
	VQ16=-(MYP6*PMA0116) + MXP6*PMA0126;
	VQ26=-(MYP6*PMA0216) + MXP6*PMA0226;
	VQ36=-(MYP6*PMA0316) + MXP6*PMA0326;
	VQ17=-(MY7*PMA0117) + MX7*PMA0127;
	VQ27=-(MY7*PMA0217) + MX7*PMA0227;
	VQ37=-(MY7*PMA0317) + MX7*PMA0327;
	Q[1]=-(GX*VQ11) - GY*VQ21;
	Q[2]=-(GX*VQ12) - GY*VQ22 - GZ*VQ32;
	Q[3]=-(GX*VQ13) - GY*VQ23 - GZ*VQ33;
	Q[4]=-(GX*VQ14) - GY*VQ24 - GZ*VQ34;
	Q[5]=-(GX*VQ15) - GY*VQ25 - GZ*VQ35;
	Q[6]=-(GX*VQ16) - GY*VQ26 - GZ*VQ36;
	Q[7]=-(GX*VQ17) - GY*VQ27 - GZ*VQ37;
}

/* *=* */
/* Number of operations : 95 '+' or '-', 146 '*' or '/' */

