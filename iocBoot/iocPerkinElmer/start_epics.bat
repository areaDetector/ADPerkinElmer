set path=%path%;C:\XIS\SDK
start medm -x -macro "P=JPHPE1:, R=cam1:, I=image1:, ROI=ROI1:, NETCDF=netCDF1:, TIFF=TIFF1:, JPEG=JPEG1:, NEXUS=Nexus1:" PerkinElmer.adl
..\..\bin\win32-x86\PerkinElmerApp st.cmd

