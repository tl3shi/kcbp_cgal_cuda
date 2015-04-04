
Prerequisites for building the Collision Detection Library under
Microsoft Visual C++ 7 or higher (tested with VS 2003/2005):

    OpenSG 1.2.0 or higher (tested up to version 1.8.0)
    Glut 3.7 (included in the above)
    

Compiling the library:

    Open the "CollisionSolution.sln" in the directory CollDet/vc7;

    Set the OPENSG variable in your system environment
    or
    Adjust the paths to OpenSG in the CollDet project properties for
    -  "Additional Include Directories" under C/C++
    -  "Additional Library Directories" under Linker.

    The 'Build Solution';
    this will create the directory CollDet/install/lib/win/Debug 
    and put the library CollDetD.dll and CollDetD.lib there.


Compiling the test programs:

    In CollDet\mak\CollisionSolution\TestSolution there is a file
    "TestSolution.sln". Open it, and change the paths to OpenSG in the project
    (i.e., test program) that you would like to build.
    Then you can build it. The executable(s) will be in CollDet\coll\test\obj\win\Debug.
    [Note: Right now, the test program intersect.exe might not compile.]


Environment variable PATH:

    Add the directories of the CollDet libraries to the PATH environment variable
    (under Settings->System Properties->Advanced).


Run a test program:

    In order to run "Interactive" or "Movem", open a DOS command prompt.
    Change directory to test\obj\win\Debug.
    Type 'Interactive.exe -x 50 -g to -a do'; this will show two tori.
	You can either move one of the tori or the viewpoint with the mouse; the
	mode can be changed by <space>.
    An exclamation mark on the console shows whether or not there is a collision.
    Type <esc> to exit.
    Run 'Interactive.exe -h' to see all options.

	Try also
		./movem -x 50 -n 10 -e 3 -v n
	for a (hopefully) nice demo.
	Note that 'movem' is really not physically-based whatsoever.
	It's just there to give you an example how to use the API.

    All other test programs are for internal purposes.
