
Prerequisites for compiling the Collision Detection package under Linux:

    OpenSG 1.2 or higher
	Glut 3.7
    g++ 3.2.2
    gmake 3.79.1


Compiling the package:


0. Set some paths:

Set the paths to OpenSG in the files
CollDet/mak/Glut.mak   and
CollDet/mak/OpenSG.mak


1. Compile Qhull:

cd CollDet/common/qhull
make optinit
make
    [now there should be libqhull.so in the directory obj/linux2_gcc_dbg/,]
    [and CollDet/install/lib/linux2 should contain a link to that lib]
cd ../..


2. Compile the CollDet library:

cd CollDet/coll
make optinit
make
    [now there should be libcoll.so in the directory obj/linux2_gcc_dgb/,]
    [and a link to it in CollDet/install/lib/linux2]


3. Compile the test programs:

cd test
make optinit
make -R
    [now there are some links to the executables in obj/linux2_gcc_dbg]


4. Set an environment variable:

Adjust your LD_LIBRARY_PATH environment variable to the new libraries, i.e.
	setenv LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:/your_path/CollDet/install/lib/linux2

On Macs:
	setenv DYLD_FALLBACK_LIBRARY_PATH .:/$HOME/Code/CollDet/install/lib/macosx:/usr/local/lib/opt

(And, of course, the libs of Glut and OpenSG must be in your LD_LIBRARY_PATH /
DYLD_FALLBACK_LIBRARY_PATH )


5. Run the test programs:

Try 
	./movem -x 50 -n 10 -e 3 -v n
for a (hopefully) nice demo.
Note that 'movem' is really not physically-based whatsoever.
It's just there to give you an example how to use the API.

The program 'interactive' can be used to interactively move one object
and have collisions checked with another one:
		./interactive -x 3 -g to -n 5 1000 -p -A
You can switch between moving the camera and the object with <space>.

The program 'bench' can be used to see the collision detection:
        ./bench -w -g file -f Data/pin.obj
or 
		./bench -x 3 -g to -n 5 1000 -d 0.5 2 -w
run it without -w to get true timings.

[All programms can be run with the option -h to get help]


Hints if things don't go smoothly:
----------------------------------

- 'make help' prints all possible "make targets"

- If you want to compile with the intel compiler just set the enviroment
  varialbe COMPILER to icc (i.e. setenv COMPILER icc). Rember to source
  iccvars before you start compiling (e.g. source /opt/intel/icc_80/bin/iccvars.csh)
 
- If you want to compile a debug version, just type 'make dbginit' instead of
  'make optinit'. Of course, this will put all .o's and executables
  in obj/platform_gcc_dbg, so that you can work on the debug- and the
  optimized version simultaneously. You don't need to change your
  LD_LIBRARY_PATH, however.

- There are also some regressions tests. You can run them in CollDet/coll/test
  with 'make tests' (the commands are in the file Commands).
  However, because of slight numerical differences among the different
  platforms, the actual output of a test can differ from the output that we
  we recorded on one of our platforms (stored in file with suffix ".out").

- If you make everything on a different platform, you need to replace 'linux2'
  by the appropriate platform name in LD_LIBRARY_PATH, of course.

- If you want to make the lib on a platform not yet supported, then you
  need to atr least create a <platform>_<compiler>.mak file in CollDet/mak,
  and you should ccheck all places in toplevel.mak and subdir.mak which
  check the platform (search for 'linux')

- It is most convenient to set LD_LIBRARY_PATH in the ~/.cshrc as follows:

---- ~/.cshrc
if ( ! $?LD_LIBRARY_PATH ) then
    setenv LD_LIBRARY_PATH .
endif
setenv LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${HOME}/CollDet/install/lib/linux2:/usr/local/lib/opt
---- ~/.cshrc

- If 'make' behaves strangely, try to run it using 'make -R'.
  Symptoms:
  Make doesn't print nice headings for each compilation step like the following one:
      -------------------------------------------------------------------------------
      --- Compiling boxtree.cpp to obj/linux2_gcc_dbg/boxtree.o
      -------------------------------------------------------------------------------
  Or/and, it doesn't find any CollDet header files.
  Explanation: in that case, for some reason, 'make' or the system does not heed the
  hash-bang line at the beginning of the Makefile. I've seen this on one Kubuntu
  installation. (If someone can tell me whe, please do so!)




// vim: set expandtab:

