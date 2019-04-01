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
R2-9 (XXX-April-2019)
----
* Fixed error in loadGainFile().  It was using the PE_CorrectionsDirectory instead of PE_GainFile.


R2-8 (2-July-2018)
----
* Changed configure/RELEASE files for compatibility with areaDetector R3-3.
* Added support for new PVs in ADCore R3-3 in opi files (NumQueuedArrays, EmptyFreeList, etc.)
* Added ADBuffers.adl to main medm screen.
* Improved op/*/autoconvert/* files with better medm files and better converters.


R2-7 (31-January-2018)
----
* Removed calls to release lock around calls to doCallbacksGenericPointer.
* Fixed medm adl files to improve the autoconversion to other display manager files.
* Added op/Makefile to automatically convert adl files to edl, ui, and opi files.
* Updated the edl, ui, and opi autoconvert directories to contain the conversions
  from the most recent adl files.


R2-6 (04-July-2017)
----
* Fixed layout of medm screen for ADCore R3-0.


R2-5 (20-February-2017)
----
* Added NDDriverVersion.
* Fixed layout of medm screen for new ADSetup.adl from ADCore R2-6.


R2-4 (29-March-2016)
----
* Added support for specifying the system index # of a PCI/PCI Express or directly connected GigE detector.
  Previously it always used the first detector found in the system.  It is now possibly to have
  multiple detectors connected to a system, and to run multiple Perkin Elmer IOCs on a single computer.  


R2-3 (22-September-2015)
----
* Added support for Data Delivered on Demand (DDD) mode. 
  There is a new record PESyncMode which allows selecting the CameraTriggerMode.  
  The most useful choices are "Framewise" and "DDD No Clear". 
  Framewise was the only mode supported in previous versions of the driver.
  * DDD mode is required to use the XRPad models from Perkin Elmer, since they only have a single
    internal frame time, it cannot be programmed.
  * DDD mode fixes significant problems in earlier versions of the driver when ImageMode=Single. 
    Previously with Single mode and TriggerMode=Internal the frame exposure actually started before
    Acquire was set to 1. 
    This means it might include some time when motors were still moving or before a shutter was open. 
    This is because the frame stream is constantly running, asynchronously to the acquisition command. 
    It just grabs the next image after acquisition is started, which can happen anytime between 
    0 and AcquireTime seconds after Acquire is set to 1. 
    Previously this could be worked around by setting PENumSkipFrames=1, but this is inefficient. 
    DDD mode allows synchronously collecting a single frame.
  * DDD mode fixes significant problems in earlier versions of the driver when TriggerMode=External.
    If trigger mode is run with PESyncMode=Framewise (the only choice in previous versions) then the
    actual acquisition time for each frame is the time between trigger pulses. 
    If this is not exactly the same as the time that was used when collecting the offset images 
    then the data quality will be poor. 
    Setting PESyncMode="DDD No Clear" allows collecting for the time specified by AcquireTime on each
    external trigger. 
    If the offset images were also collected for this time then the data quality will be
    good, even if the time between triggers is varying or is different from AcquireTime. 
    Note however that DDD mode has an additional overhead, so the maximum frame rate is less than 
    in Framewise mode.
    The overhead is about 0.2 seconds on the model 1621 in 2048x2048 (2K) mode, 
    and about 0.1 seconds in 1K mode.
    The maximum frame rates are thus reduced from 15Hz to 5Hz in 2K mode and from 
    30Hz to 10Hz in 1K mode.
    Framewise mode is thus still useful for fast triggered acquisition as long as the offset images 
    are collected for the same time as the time between trigger pulses.
* The driver code was significantly rewritten to add the DDD mode support, make it simpler, 
  and to add debugging information on calls to the XIS library.

R2-2 (17-April-2015)
----
* Added PEOffsetConstant.  This is a user-specified value that is added to each pixel
  when offset correction is done.  It adds a pedestal to the data, ensuring that offset
  correction does not result in negative values, which are then clipped to 0.  Values
  of 100-200 should be sufficient to prevent negative data during offset correction.
* Updated to version 3-3-2-3 of the XIS SDK.
* Bug fix: The driver was not changing the gain when the PEGain record was processed.
* Bug fix: If the binning was changed while offset correction was enabled, and without
  collecting a new offset image, it could crash the IOC.  Now if the binning is changed
  the offset correction is set to Unavailable until a new offset image is collected.


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

