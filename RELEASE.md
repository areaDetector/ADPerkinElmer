ADPerkinElmer Releases
======================

The latest untagged master branch can be obtained at
https://github.com/areaDetector/ADPerkinElmer.

Tagged source code and pre-built binary releases prior to R2-0 are included
in the areaDetector releases available via links at
http://cars.uchicago.edu/software/epics/areaDetector.html.

Tagged source code releases from R2-0 onward can be obtained at 
https://github.com/areaDetector/ADPerkinElmer/releases.

Tagged prebuilt binaries from R2-0 onward can be obtained at
http://cars.uchicago.edu/software/pub/ADPerkinElmer.

The versions of EPICS base, asyn, and other synApps modules used for each release can be obtained from 
the EXAMPLE_RELEASE_PATHS.local, EXAMPLE_RELEASE_LIBS.local, and EXAMPLE_RELEASE_PRODS.local
files respectively, in the configure/ directory of the appropriate release of the 
[top-level areaDetector](https://github.com/areaDetector/areaDetector) repository.


Release Notes
=============

R2-1 (16-April-2015)
----
* Fixed a bug introduced in R1-8.  The "maxMemory" and "priority" arguments in the PerkinElmerConfig
  iocsh command were not being set correctly. Uninitialized memory locations were being passed to 
  the constructor, which could lead to unpredictable behavior.
* Bug fix: running asynReport with details > 1 would crash IOC if acquisition was in progress.
* Changes in st.cmd to be compatible with ADCore R2-2


R2-0 (20-March-2014)
----
* Moved the repository to [Github](https://github.com/areaDetector/ADmarCCD).
* Re-organized the directory structure to separate the driver library from the example IOC application.


R1-9-1 and earlier
------------------
Release notes are part of the
[areaDetector Release Notes](http://cars.uchicago.edu/software/epics/areaDetectorReleaseNotes.html).

Future Releases
===============
* Test gain file input
* Test bad pixel map, modify ours
* Test supplied Nexus template file

