[comment]: <> (# AnyFeature-VSLAM)

<!-- PROJECT LOGO -->

<p align="center">
  <h1 align="center"> AnyFeature-VSLAM  (RSS 2024)
  <h3 align="center"> AnyFeature-VSLAM: Automating the Usage of Any Chosen Feature into Visual SLAM</h3>  
  </h1>
  <p align="center">
    <a href="https://scholar.google.com/citations?user=SDtnGogAAAAJ&hl=en&oi=a"><strong>Alejandro Fontan</strong></a>
    ·
    <a href="https://scholar.google.com/citations?user=j_sMzokAAAAJ&hl=en&oi=a"><strong>Javier Civera</strong></a>
    ·
    <a href="https://scholar.google.com/citations?user=TDSmCKgAAAAJ&hl=en&oi=ao"><strong>Michael Milford</strong></a>
  </p>

**Any-Feature V-SLAM** is an automated visual SLAM library for **Monocular** cameras capable of switching to a chosen type of feature effortlessly and without manual intervention. 

**AnyFeature-VSLAM runs seamlessly as a baseline for experiments within the  [VSLAM-LAB](https://github.com/alejandrofontan/VSLAM-LAB) framework.** 

**Note:** More updates from the publication [**RSS 2024 AnyFeature-VSLAM**](https://roboticsconference.org/program/papers/84/) will be available in the coming weeks. Extensions to **RGB-D** and **Stereo** cameras are coming soon, with future work planned for extensions to **Visual Inertial** systems.

**Acknowledgments to:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) ([ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)), [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)), Carlos Campos, Richard Elvira and Juan J. Gómez Rodríguez ([ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)).


# Getting Started
If you want to run AnyFeature-VSLAM on your own data that is not contained in  [**VSLAM-LAB**](https://github.com/alejandrofontan/VSLAM-LAB), you can follow these steps.

To ensure all dependencies are properly installed, we suggest using *mamba*. If you haven't installed *mamba* yet, please download and set up [`miniforge`](https://github.com/conda-forge/miniforge), which is a more streamlined installer. This installer will create a "base" environment that includes the *conda* and *mamba* package managers.

If you already have a conda installation, you can add mamba by running:
```
conda install mamba -c conda-forge
```

## Installation
Clone the repository:
```
git clone --recurse-submodules https://github.com/alejandrofontan/AnyFeature-VSLAM.git && cd AnyFeature-VSLAM
```
Create the environment:
```
mamba env create -f environment.yml && mamba activate anyfeature
```
Build AnyFeature-VSLAM:
```
./build.sh
```

## Usage

**AnyFeature-VSLAM** runs seamlessly in sequences from [**VSLAM-LAB**](https://github.com/alejandrofontan/VSLAM-LAB). If you want to run your own sequences you can format your data as follows. A sequence folder containing 1) an 'rgb' folder with the images, 2) an rgb.txt file with timestamps and paths to the images, and 3) a calibration.yaml file. Try the following toy example:
```
./bin/mono "sequence_path:docs/toy_sequence"
```
Additional options:
```
"Feat:orb32" (orb32/akaze61/brisk48/sift128/anyfeatbin/anyfeatnonbin)
"Vis:1"
"Voc:anyfeature_vocabulary"
"exp_folder:""
"exp_id:0"
"FixRes:0"
```

# Add your own feature

## Create the Vocabulary
```
./createVocabulary.sh
python createVocabulary.py --feature_id 0 --rgb_folder ~/BOVISA/2008-09-01/rgb --featSettYamlFile settings/orb32_settings.yaml
```

## License
**AnyFeature-VSLAM** builds on [**ORB-SLAM2**](https://github.com/raulmur/ORB_SLAM2) and inherits its release under a [GPLv3 license](https://github.com/alejandrofontan/AnyFeature-VSLAM/blob/main/docs/License-gpl.txt). For a list of all other code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/alejandrofontan/AnyFeature-VSLAM/blob/main/docs/Dependencies.md).

If you found this code/work to be useful in your own research, please considering citing the following:
```bibtex
@inproceedings{fontanRSS2024,
  title={{AnyFeature-VSLAM}: Automating the Usage of Any Chosen Feature into Visual SLAM},
  author={Fontan, Alejandro and Civera, Javier and Milford, Michael},
  booktitle={Robotics: Science and Systems},
  year={2024}
}

```

## Related Publications:
[AnyFeature-VSLAM] Alejandro Fontan, Javier Civera and Michael Milford, **Automating the Usage of Any Chosen Feature into Visual SLAM**, *Robotics: Science and Systems, 2024*. **[PDF](https://roboticsconference.org/program/papers/84/)**.

[ORB-SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, *IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021*. **[PDF](https://arxiv.org/abs/2007.11898)**.

[ORB-SLAM2] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[ORB-SLAM] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**
