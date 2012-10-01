---------------------------------------------------------------------------------------------------------------------------
// If you are using this software for your research please site out paper 'Change detection using Airborne Lidar -
// Applications to Earthquakes. Aravindhan K Krishnan, Edwin Nissen, Srikanth Saripalli, Ramon Arrowsmith. International
// Symposium on Experimental Robotics, 2012'
---------------------------------------------------------------------------------------------------------------------------

This source code is part of ongoing research and may not be stable. The code was developed on Linux (Ubunutu) and is
recommended for use ONLY on Linux platforms.

This software has the following dependencies
1) PCL (pointclouds.org)
2) boost (boost.org)

The above libraries must be installed before compiling this software.

Installing PCL from the source is MANDATORY. Please follow the instructions on
http://pointclouds.org/downloads/source.html to compile PCL from the source. 

Few changes have to be made to the PCL source which is required for this
software to compile. Follow the instructions in the document 'pcl_changes.txt' to make the appropriate changes to PCL.


The build tool used to compile this software is cmake. Please install cmake before you start compiling.

HOW TO COMPILE
==============

1) tar -xvjf registration.tar.gz (Extracts the tar file. This creats a folder named registration)
2) cd registration (This command takes you to the registation folder)
3) mkdir build (Creates a directory named build within the registration folder)
4) cd build (Moves to the build folder you just created)
5) cmake .. (Generates the files required for compiling the software)
6) make (Compiles the software)

The following files will now be created in the build folder
1) registration
2) texttopcd
3) get_displacements

Please read the document 'HOWTO.txt' to learn how to use these programs.


ACKNOWLEDGEMENTS
================

I thank my advisors Prof Srikanth Saripalli and Prof Ramon Arrowsmith and my colleague Dr. Edwin Nissen for their support
and valuable inputs. Special thanks to Dr. Edwin Nissen for running numerous experiments on this software and
validating the results, thereby helping me identify bugs and fix them.

This work was supported by the US National Science Foundation EarthScope Program grant (EAR-1148302) and the Southern California Earthquake Center.
