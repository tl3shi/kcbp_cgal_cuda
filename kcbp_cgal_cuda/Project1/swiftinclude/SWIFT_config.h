/*************************************************************************\

  Copyright 2000 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify OR distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Ehmann, M. Lin
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919) 962-1749

  EMail:               geom@cs.unc.edu
                       ehmann@cs.unc.edu
                       lin@cs.unc.edu

\**************************************************************************/


//////////////////////////////////////////////////////////////////////////////
//
// SWIFT_config.h
//
// Description:
//      Compile time configuration.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _SWIFT_CONFIG_H_
#define _SWIFT_CONFIG_H_

#define SWIFT_VERSION1_1_03

///////////////////////////////////////////////////////////////////////////////
// SWIFT Compile Configuration
///////////////////////////////////////////////////////////////////////////////

// Set what type of floating point numbers to use in SWIFT.  If SWIFT_USE_FLOAT
// is defined then float's are used, otherwise doubles are used.  Doubles are
// recommended due to higher accuracy.
#define SWIFT_USE_FLOAT

// The maximum valence of any vertex that will be given to SWIFT.
#define SWIFT_MAX_VALENCE 100

// If SWIFT_ALWAYS_LOOKUP_TABLE is defined, then a lookup table is always used
// to initialize the distance minimization (coherence is not taken advantage of
// by initializing with the previous feature pairs).  The hierarchy should NOT
// be turned on if the lookup table is on.
//#define SWIFT_ALWAYS_LOOKUP_TABLE

// Turn one of these on for the resolution of the lookup table to use.  Even
// if SWIFT_ALWAYS_LOOKUP_TABLE is not defined, lookup tables are still created
// (used to initialize in some cases).  So, one of these must always be on.  The
// resolutions are 22.5 degrees, 11.25 degrees, and 5.625 degrees.  The storage
// costs are approximately 0.5 kB, 2 kB, and 8 kB respectively.  There is not
// much of a performance difference but in general, the highest resolution
// provides the highest performance.
#define SWIFT_LUT_RESOLUTION_22_5
//#define SWIFT_LUT_RESOLUTION_11_25
//#define SWIFT_LUT_RESOLUTION_5_625

// If SWIFT_HIERARCHY is defined, then each convex piece receives a hierarchy
// around it.  If not, no hierarchy exists for any of the convex pieces.
// The lookup table should NOT be turned on if the hierarchy is on.
//#define SWIFT_HIERARCHY

// If SWIFT_QSLIM_HIER is defined then the QSlim hierarchy is used.  The qslim
// and the qhull libraries must be linked into the application and SWIFT's
// Makefile must be modified to be able to include qhull and qslim headers.
// If this is off, the internal Dobkin-Kirkpatrick hierarchy is used.  The QSlim
// hierarchy performs better in general.  If SWIFT_HIERARCHY is not defined,
// this option has no effect.
//#define SWIFT_QSLIM_HIER

// Debugging option.  This may be useful if you are having a problem using the
// system and need to have some automatic checking done on imported geometry
// and certain function calls.  Error messages (if they occur) will be written
// to stderr.
//#define SWIFT_DEBUG

///////////////////////////////////////////////////////////////////////////////
// Disable Warnings
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Error Checking
///////////////////////////////////////////////////////////////////////////////

# if defined(SWIFT_HIERARCHY) && defined(SWIFT_ALWAYS_LOOKUP_TABLE)
#  error : Compile Configuration Error: Cannot have both the hierarchy and the lookup table turned on
# endif

#if !defined(LUT_RESOLUTION_22_5) && !defined(LUT_RESOLUTION_11_25) && !defined(LUT_RESOLUTION_5_625)
# define LUT_RESOLUTION_22_5
#endif

# ifndef SWIFT_MAX_VALENCE
#  define SWIFT_MAX_VALENCE 100
# endif

#endif

 
