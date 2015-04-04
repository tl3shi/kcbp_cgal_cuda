
/*****************************************************************************\
 *                              topology
\*****************************************************************************/

/*! @file 
 *
 *  @brief
 *    Testprogram for class Topology.
 *
 *  @author Gabriel Zachmann
 *  
 *  @todo
 *    topology.out neu erzeugen, wenn OSG's Kugel in Ordnung ist, d.h.,
 *    dass Punkte nicht mehrfach vorkommen.
 */


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>

#include <ColTopology.h>
#include <ColUtils.h>

#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGSimpleGeometry.h>


//**************************************************************************
// Utility functions
//**************************************************************************

enum GeometryTypeE { OBJ_PLANE, OBJ_SPHERE, OBJ_TORUS, OBJ_BOX };

osg::GeometryPtr createGeom( GeometryTypeE type, unsigned int complexity )
{
	osg::NodePtr node;
	osg::GeometryPtr geom;

    switch ( type )
    {
        case OBJ_PLANE:
            node = osg::makePlane( 1, 1, complexity, complexity );
            break;

        case OBJ_SPHERE:
            complexity = static_cast<int>( logf( complexity*complexity ) );
            node = osg::makeSphere( complexity, 0.5 );
            break;

        case OBJ_TORUS:
            node = osg::makeTorus( 0.35, 0.5, complexity+1, complexity+1 );
            break;

        case OBJ_BOX:
            node = col::makeCube( 0.5, GL_QUADS );
            break;

        default:
            fputs("createGeom: BUG: type is out of range!\n",stderr);
            exit(-1);
    }

    if ( node == osg::NullFC )
    {
        fprintf(stderr,"topology: createGeom failed!\n");
        exit(-1);                   // makes no sense to continue
    }

    geom  = osg::GeometryPtr::dcast( node->getCore() );

    if ( geom == osg::NullFC )
    {
        fprintf(stderr,"createGeom: dcast(GeometryPtr) returned NULL!\n");
        exit(-1);
    }

	return geom;
}


void printPoints( osg::GeometryPtr geom )
{
	const osg::MFPnt3f *points = col::getPoints( geom );
	for ( unsigned int i = 0; i < points->size(); i ++ )
		col::printPnt( (*points)[i] );
}




//**************************************************************************
// main
//**************************************************************************


int main( void )
{
	osg::osgInit( 0, NULL );

	osg::Pnt3f pnt[8] = { Pnt3f(-1,-1, 1),
						  Pnt3f( 1,-1, 1),
						  Pnt3f( 1, 1, 1),
						  Pnt3f(-1, 1, 1),
						  Pnt3f(-1,-1,-1),
						  Pnt3f( 1,-1,-1),
						  Pnt3f( 1, 1,-1),
						  Pnt3f(-1, 1,-1)
						};
    unsigned int face[6][col::Dop::NumOri] = { {0,1,2,3},
										  {1,5,6,2},
										  {2,6,7,3},
										  {0,3,7,4},
										  {0,4,5,1},
										  {4,7,6,5}
										};
    unsigned int face_nv[6] = { 4, 4, 4, 4, 4, 4 };

	col::Topology topo1( face, 6, face_nv );

	puts("\nCube from array:\n");
	puts("points:");
	for ( int i = 0; i < 8; i ++ )
		col::printPnt( pnt[i] );
	puts("topology:");
	topo1.print();

	osg::GeometryPtr geom;

	geom = createGeom( OBJ_PLANE, 3 );
	col::Topology topo2( geom );

	puts("\nPlane:\n");
	puts("points:");
	printPoints( geom );
	puts("topology:");
	topo2.print();

	geom = createGeom( OBJ_SPHERE, 1 );
	col::Topology topo3( geom );

	puts("\nSphere:\n");
	puts("points:");
	printPoints( geom );
	puts("topology:");
	topo3.print();

	geom = createGeom( OBJ_TORUS, 3 );
	col::Topology topo4( geom, true, 1E-4 );

	puts("\nTorus with vertex unification:\n");
	puts("points:");
	printPoints( geom );
	puts("topology:");
	topo4.print();

	geom = createGeom( OBJ_TORUS, 3 );
	topo4.createFromGeom( geom, false, 0 );

	puts("\nTorus without vertex unification:\n");
	puts("points:");
	printPoints( geom );
	puts("topology:");
	topo4.print();

	geom = createGeom( OBJ_BOX, 0 );
	col::Topology topo5( geom );		// does not do unification by default
	puts("\nBox:\n");
	puts("points:");
	printPoints( geom );
	puts("topology:");
	topo5.print();

	return 0;
}


char cvsid[] =
"@(#)$Id: topology.cpp,v 1.6 2004/03/09 13:41:36 ehlgen Exp $";



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
 *   Beschreibung ..
 *
 * @warning
 *   Dinge, die der Aufrufer unbedingt beachten muss...
 *
 * @pre
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
 * @implementation
 *   Implementierungsdetails, TODOs, ...
 *
 **/




