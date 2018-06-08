==============
=== CREDIT ===
==============

Basic 2D monocular visual odometry assuming a bottom-looking camera and flat floor.

Author : Antoni Burguera
e-mail : antoni.burguera@uib.es

This software was developed and used in a paper currently under review. Please, let us know if you plan to use this software so we can cite us with the proper reference.

=========================
=== BUILD THE PACKAGE ===
=========================

The software is pure matlab code. Just put it wherever you want and use it.

However, the code uses the VLFeat library. If you don't have VLFeat, download it from:

http://www.vlfeat.org/

Depending on how you install VLFeat, it may be necessary to run (from matlab) vl_setup.m from the vlfeat/toolbox directory.

====================
=== RUN THE CODE ===
====================

The code assumes that the images to be used are named with consecutive numbers and are png. Let's assume that the relative path to the images is "video", and that in this directory there are images named from 1.png to 98.png. Let us also assume that we want to compute odometry from image 5.png to image 90.png. Finally, le us assume that we want to scale the images so that the width is 320 pixels. To do so, just type (from matlab command line):

>> odoData=compute_odometry('video/',5,90,320);

It is easy to change the code to accept other image formats and image file naming conventions. Just check and modify the function get_image within compute_odometry.m

The output format is:

odoData - Array of structures so that:
  * odoData(i).f: SIFT features within the corresponding image.
  * odoData(i).d: SIFT descriptors within the corresponding image.
  * odoData(i).X: Motion (x,y,o)' from image corresponding to odoData(i-1) to image corresponding to odoData(i).
    odoData(0).X is set to (0,0,0)'.
  * odoData(i).imNum: the number of the image. For example, imNum=57 means that f and d are features and descriptors of the image named 57.png and that X is the estimated motion from 56.png to 57.png

After obtaining odoData, the estimated trajectory can be displayed by doing

>> plot_odometry(odoData)

=======================
=== TROUBLESHOOTING ===
=======================

This software package has been tested using Matlab 9.0.0.341360 (R2016a) running on Ubuntu 16.04LTS. Other configurations are untested.