// Filename:  kukaKDL.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: 
//

#ifndef KUKAKDL_CONST_HPP
#define KUKAKDL_CONST_HPP

#define MY_PI  3.14159265359
#define sign(x) (x<0.0 ? -1.0 : (x==0.0 ? 0.0 : 1.0))

// From the paper [Jubien 2014]
//Geometric parameters ([Khalil and Kleinfinger 1986] notation)
#define ALPHA1 0.
#define ALPHA2 (MY_PI/2.0)
#define ALPHA3 (-MY_PI/2.0)
#define ALPHA4 (-MY_PI/2.0)
#define ALPHA5 (MY_PI/2.0)
#define ALPHA6 (MY_PI/2.0)
#define ALPHA7 (-MY_PI/2.0)

#define R1 0.
#define R2 0.
#define R3 0.4
#define R4 0.
#define R5 0.39
#define R6 0.
#define R7 0.

#define XX1 0.
#define XY1 0.
#define XZ1 0.
#define YY1 0.
#define YZ1 0.
#define ZZ1 3.20
#define MX1 0.
#define MY1 0.
#define MZ1 0.
#define M1  0.
#define IA1 0.
#define XX2 1.31
#define XY2 0.
#define XZ2 0.
#define YY2 0.
#define YZ2 0.
#define ZZ2 4.46
#define MX2 0.
#define MY2 3.37
#define MZ2 0.
#define M2  0.
#define IA2 0.
#define XX3 0.
#define XY3 0.
#define XZ3 0.
#define YY3 0.
#define YZ3 0.
#define ZZ3 0.0108
#define MX3 0.
#define MY3 0.
#define MZ3 0.
#define M3  0.
#define IA3 2.01
#define XX4 0.368
#define XY4 0.491
#define XZ4 0.
#define YY4 0.
#define YZ4 0.
#define ZZ4 0.
#define MX4 0.
#define MY4 -1.37
#define MZ4 0.
#define M4  0.
#define IA4 1.92
#define XX5 0.
#define XY5 0.
#define XZ5 0.
#define YY5 0.
#define YZ5 0.
#define ZZ5 0.00633
#define MX5 0.
#define MY5 0.049
#define MZ5 0.
#define M5  0.
#define IA5 0.776
#define XX6 0.
#define XY6 0.
#define XZ6 0.
#define YY6 0.
#define YZ6 0.
#define ZZ6 0.00377
#define MX6 0.
#define MY6 0.042
#define MZ6 0.
#define M6  0.
#define IA6 0.391
#define XX7 0.
#define XY7 0.
#define XZ7 0.
#define YY7 0.
#define YZ7 0.
#define ZZ7 0.000120
// These 4 have to remain 0 (the tool inertia is accounted for in the code)   
#define MX7 0.
#define MY7 0.
#define MZ7 0.
#define M7  0.
//
#define IA7 0.399
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
#define GX  0.
#define GY  0.
#define GZ	-9.81			

#endif


