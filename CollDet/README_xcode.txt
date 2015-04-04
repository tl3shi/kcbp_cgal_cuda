
The Xcode project files reside in CollDet/xcode;
the executables are still placed in the source code directories,
libraries are put in CollDet/install/lib/macosx,
and the intermediate files (.o) are placed in the obj/xcode subdirs.

How to build:
1. Build qhull using qhull.xcodeproj
2. Build the CollDet library using coll.xcodeproj
3. Build the test programs buy building the other targets in the same
   project

Caveat:
  The project files are setup for intel-mac's.

How to run the example programs in coll/test:
  You can run 'interactive', 'intersect', and 'movem' from Xcode.
  I have defined some sensible command line arguments and the env. variable
  DYLD_FALLBACK_LIBRARY_PATH.
  Hint: move the mouse and press space.
  Note: you will have to adust the setting of DYLD_FALLBACK_LIBRARY_PATH
  in each executable environment!

  For more info on command line arguments type 'interactive -h' in a shell.

The other test programs are meant for regression testing.
You can try 'make tests' to see whether they pass.

