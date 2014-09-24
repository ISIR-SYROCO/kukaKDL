// Filename:  kukaKDL.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: 
//

#ifndef KUKAKDL_CONST_HPP
#define KUKAKDL_CONST_HPP

#define MY_PI  3.14159265359
#define sign(x) (x<0.0 ? -1.0 : (x==0.0 ? 0.0 : 1.0))

// Kinematics parameters of the KUKA LWR
#define ALPHA0 0.
#define ALPHA1 (MY_PI/2.0)
#define ALPHA2 (-MY_PI/2.0)
#define ALPHA3 (-MY_PI/2.0)
#define ALPHA4 (MY_PI/2.0)
#define ALPHA5 (MY_PI/2.0)
#define ALPHA6 (-MY_PI/2.0)
#define ALPHA7 0.

#define R0 0.31
#define R1 0.
#define R2 0.4
#define R3 0.
#define R4 0.39
#define R5 0.
#define R6 0.
#define R7 0.078

// Gravity Vector
#define GX  0.
#define GY  0.
#define GZ	-9.81	

// Dynamic parameters of the KUKA LWR from Leuven
// Used to compute the dynamic model using KDL

#define XX1_KDL 0.
#define XY1_KDL 0.
#define XZ1_KDL 0.
#define YY1_KDL 0.
#define YZ1_KDL 0.
#define ZZ1_KDL 0.0115343
#define COGX1_KDL 0.
#define COGY1_KDL 0.
#define COGZ1_KDL 0.
#define M1_KDL  2.

#define XX2_KDL -0.5471572
#define XY2_KDL 0.
#define XZ2_KDL 0.
#define YY2_KDL -0.0000302
#define YZ2_KDL 0.0018828
#define ZZ2_KDL -0.5423253
#define COGX2_KDL 0.
#define COGY2_KDL -0.31205011
#define COGZ2_KDL -0.0038871
#define M2_KDL 2.

#define XX3_KDL 0.0063507
#define XY3_KDL 0.
#define XZ3_KDL 0.
#define YY3_KDL 0.
#define YZ3_KDL -0.0005147
#define ZZ3_KDL 0.0107804
#define COGX3_KDL 0.
#define COGY3_KDL -0.0015515
#define COGZ3_KDL 0.
#define M3_KDL 2.

#define XX4_KDL -1.0436952
#define XY4_KDL 0.
#define XZ4_KDL 0.
#define YY4_KDL 0.
#define YZ4_KDL 0.0005324
#define ZZ4_KDL -1.0392780
#define COGX4_KDL 0.
#define COGY4_KDL 0.5216809
#define COGZ4_KDL 0.
#define M4_KDL 2.

#define XX5_KDL 0.0036654
#define XY5_KDL 0.
#define XZ5_KDL 0.
#define YY5_KDL 0.
#define YZ5_KDL 0.0004226
#define ZZ5_KDL 0.0060429
#define COGX5_KDL 0.
#define COGY5_KDL 0.0119891
#define COGZ5_KDL 0.
#define M5_KDL 2.

#define XX6_KDL 0.0010431
#define XY6_KDL 0.
#define XZ6_KDL 0.
#define YY6_KDL 0.
#define YZ6_KDL 0.0000101
#define ZZ6_KDL 0.0036376
#define COGX6_KDL 0.
#define COGY6_KDL 0.0080787
#define COGZ6_KDL 0.
#define M6_KDL 2.

#define XX7_KDL 0.000001
#define XY7_KDL 0.
#define XZ7_KDL 0.
#define YY7_KDL 0.
#define YZ7_KDL 0.
#define ZZ7_KDL 0.0001203
#define COGX7_KDL 0.
#define COGY7_KDL 0.
#define COGZ7_KDL 0.
#define M7_KDL 2.	

// Values from IRCCYN (use with great caution)
#define FS1 11.9
#define FV1 14.4
#define FS2 11.5  
#define FV2 15.3
#define FS3 8.98
#define FV3 6.55
#define FS4 8.35
#define FV4 11.0
#define FS5 8.31
#define FV5 4.29
#define FS6 4.72
#define FV6 2.26
#define FS7 6.04
#define FV7 1.6

#endif


