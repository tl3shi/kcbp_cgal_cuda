
/*****************************************************************************\
 *                Timing of some simple OSG things.
\*****************************************************************************/
/*! @file
 *
 *  @brief
 *    Timing of some simple OSG things.
 *
 *  @author Gabriel Zachmann
 *
 *  @flags
 *    @arg @c YY - compiles without ..
 *
 */



//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <OpenSG/OSGConfig.h>

#include <OpenSG/OSGBaseFunctions.h>
#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGGeoProperty.h>
#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGQuaternion.h>
#include <OpenSG/OSGMatrix.h>
#include <OpenSG/OSGTransform.h>
#include <OpenSG/OSGGroup.h>
#include <OpenSG/OSGBoxVolume.h>

#include <OpenSG/OSGTrackball.h>

#include <ColUtils.h>

using osg::beginEditCP;
using osg::endEditCP;
using osg::Vec3f;
using osg::Pnt3f;


/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/

namespace {

const int NPoints = 1000;
const int NLoops  = 10000000;

osg::NodePtr		root;

typedef float c_pointT[3];

}


/***************************************************************************\
 *                        Tests                                            *
\***************************************************************************/

void timing1( )
{
	osg::NodePtr trian_node1 = osg::Node::create();
	osg::GeometryPtr trian1 = osg::Geometry::create();
	osg::GeoPositions3f::PtrType pnts1 = osg::GeoPositions3f::create();
	c_pointT c_points[NPoints];

	double f;
	float tim, start;
	int j;


	printf("%d times writing coords of a point (p[i][0|1|2] = float)\n",
		   NLoops );

	// fill OSG data structures
	beginEditCP( trian_node1 );
	trian_node1->setCore( trian1 );
	endEditCP( trian_node1 );
	root->addChild( trian_node1 );

	trian1->setPositions( pnts1 );
	osg::MFPnt3f& p = *pnts1->getFieldPtr();
	beginEditCP(pnts1);
	for ( int i = 0; i < NPoints; i ++ )
		p.addValue( Pnt3f(-1, -1, -1) );
	endEditCP(pnts1);

	// fill C data structures
	for ( int i = 0; i < NPoints; i ++ )
		c_points[i][0] = -1.0,
		c_points[i][1] = -1.0,
		c_points[i][2] = -1.0;

	// OSG timing
	start = col::time();
	j = 0;
	f = 1;
	for ( int i = 0; i < NLoops; i ++ )
	{
		p[j][0] = f; p[j][1] = f; p[j][2] = f;
		f *= 1.00000001;
		j ++ ;
		if ( j >= NPoints )
			j = 0;
	}
	tim = col::time() - start;

	printf("OSG: %f millisec utime\n", tim );

	// C timing
	start = col::time();
	j = 0;
	f = 1;
	for ( int i = 0; i < NLoops; i ++ )
	{
		c_points[j][0] = f; c_points[j][1] = f; c_points[j][2] = f;
		f *= 1.00000001;
		j ++ ;
		if ( j >= NPoints )
			j = 0;
	}
	tim = col::time() - start;

	printf("C  : %f millisec utime\n", tim );

	// make sure nothing gets optimized away
	f = 0;
	j = 0;
	for ( int i = 0; i < NLoops; i ++ )
	{
		f += p[j][0] + p[j][1] + p[j][2];
		j ++ ;
		if ( j >= NPoints )
			j = 0;
	}
	j = 0;
	for ( int i = 0; i < NLoops; i ++ )
	{
		f += c_points[j][0] + c_points[j][1] + c_points[j][2];
		j ++ ;
		if ( j >= NPoints )
			j = 0;
	}

	printf("f = %f\n", f );
}




/***************************************************************************\
 *                              Main                                       *
\***************************************************************************/



void commandlineerror( char *cmd, char *parm )
{
	if ( cmd )
		fprintf(stderr, "Offending option: %s\n", cmd );
	if ( parm )
		fprintf(stderr, "with first parameter: %s\n", parm );

	fprintf(stderr, "\n\nUsage: osg options ...\n"
	"-h          this help menu\n"
	"\n\n" );

	if ( cmd )
		exit(-1);				// problem occured
	else
		exit(0);				// was option -h
}


void parsecommandline( int argc, char *argv[] )
{
	/* valid option characters; last char MUST be 0 ! */
	char optionchar[] =   { 'h', 0 };
	int musthaveparam[] = {  0 ,  0 };
	int nopts;
	int mhp[256];
	int isopt[256];
	int i;
	char optchar;


	nopts = strlen(optionchar);
	if ( nopts > 50 )
	{
		fprintf(stderr, "\n\nparsecommandline: the option-chars string "
				"seems to be\nVERY long (%d bytes) !\n\n", nopts );
		exit(-1);
	}

	bzero( isopt, 256*sizeof(int) );
	bzero( mhp, 256*sizeof(int) );
	for ( i = 0; i < nopts; i ++ )
	{
		if ( isopt[optionchar[i]] )
		{
			fprintf(stderr, "\n\nparsecommandline: Bug: an option character is"
				   " specified twice in the\n"
				   "option character array !\n\n");
			exit(-1);
		}
		isopt[ optionchar[i] ] = 1;
		mhp[ optionchar[i] ] = musthaveparam[i];
	}

	++argv; --argc;
	while ( argc > 0 )
	{
		if ( argv[0][0] == '-' )
		{
			optchar = argv[0][1];

			if ( ! isopt[optchar] )
			{
				fprintf(stderr, "\nIs not a valid command line option\n");
				commandlineerror( argv[0], NULL );
			}
			for ( i = 0; i < mhp[optchar]; i ++ )
				if ( ! argv[1+i] || argv[1+i][0] == '-' )
				{
					fprintf(stderr, "\nCommand line option -%c must "
							"have %d parameter(s)\n",
							argv[0][1], mhp[optchar] );
					commandlineerror( argv[0], NULL );
					argv += 1 + i;
					argc -= 1 + i;
					continue;
				}

			switch ( optchar )
			{
				/* '-h' doesn't have an optional parameter */
				case 'h': commandlineerror( NULL, NULL);  break;

				default: fprintf(stderr, "\nBug in parsecommandline !\n");
						 commandlineerror( argv[0], NULL );
			}

			argv += 1 + mhp[optchar];
			argc -= 1 + mhp[optchar];
		}
		else
		{
			/* command line arg doesn't start with '-' */
			fprintf(stderr, "\nThis is not a valid command line option\n");
			commandlineerror( argv[0], NULL );
			/* or, load file instead .. */
		}
	}
}



int main( int argc, char *argv[] )
{

	// OSG init
	osg::osgLog().setLogLevel(osg::LOG_WARNING);
	osg::osgInit(argc, argv);

	// parse command line options
	parsecommandline( argc, argv );

	// create the graph
	osg::NodePtr node;

	// root
	root = osg::Node::create();
	beginEditCP(root);
	root->setCore( osg::Group::create() );

	// finish scene graph
	endEditCP(root);

	timing1();

	return 0;
}


static char cvsid[] =
"@(#)$Id: osg.cpp,v 1.2 2002/09/20 15:02:57 zach Exp $";



/**  Zum Abpasten fuer eine Funktion, Methode, oder Makro-"Funktion".
 *
 * @param param1	Comment for param1
 *
 * @return
 *   Text for return value.
 *
 * Detaillierte Beschreibung ...
 *
 * @throw Exception
 *  Beschreibung ..
 *
 * @warning
 *   Dinge, die der Aufrufer unbedingt beachten muss...
 *
 * @assumptions
 *   Annahmnen, die die Funktion macht...
 *
 * @sideeffects
 *   Nebenwirkungen, globale Variablen, die veraendert werden, ..
 *
 * @todo
 *   Was noch getan werden muss
 *
 * @bug
 *   Bekannte Bugs dieser Funktion
 *
 * @see
 *   ...
 *
 * @internal
 *   Implementierungsdetails, TODOs, ...
 *
 **/



