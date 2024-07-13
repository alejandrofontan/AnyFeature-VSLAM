## List of Known Dependencies
### AnyFeature-VSLAM

In this document we list all the pieces of code included  by AnyFeature-VSLAM and linked libraries which are not property of the authors of AnyFeature-VSLAM.

##### Code in ORB-SLAM2
Please follow the license of ORB-SLAM2.

License URL: https://github.com/raulmur/ORB_SLAM2/blob/master/LICENSE.txt

##### Code in Thirdparty folder

* All code in **DBoW2** folder.
This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) library. All files included are BSD licensed.

##### Code in Anaconda

* **g2o**
This is a modified version of [g2o](https://github.com/RainerKuemmerle/g2o). All files included are BSD licensed.

* **Pangolin (visualization and user interface)**.
[MIT license](https://en.wikipedia.org/wiki/MIT_License).

* **OpenCV**.
BSD license.

* **Binary Robust Invariant Scalable Keypoints**.
[BSD-3 license](https://github.com/gwli/brisk/blob/master/LICENSE).

* **Accelerated-KAZE Features**.
[BSD-3 license](https://github.com/pablofdezalc/akaze/blob/master/LICENSE).

* **SiftGPU**.
[License](https://github.com/pitzer/SiftGPU/blob/master/license.txt).
  
* **Speeded Up Robust Features**
We use the popular algorithm for 2d feature detection in OpenCV, [SURF](https://docs.opencv.org/4.x/d2/dca/group__xfeatures2d__nonfree.html), that is known to be patented. You need to set the OPENCV_ENABLE_NONFREE option in cmake to use those. Use them at your own risk.


##### Library dependencies 

* **Eigen3**.
For versions greater than 3.1.1 is MPL2, earlier versions are LGPLv3.





