#!/bin/bash

delete_if_exists() {
  local folder=$1
  build_folder="${folder}/build"
  bin_folder="${folder}/bin"
  lib_folder="${folder}/lib"
  if [ -d "$build_folder" ]; then
    rm -rf "$build_folder"
  fi
  if [ -d "$bin_folder" ]; then
    rm -rf "$bin_folder"
  fi
  if [ -d "$lib_folder" ]; then
    rm -rf "$lib_folder"
  fi
}

# Check inputs
force_build=false
verbose=false
for input in "$@"
do
    echo "Processing input: $input"
    if [ "$input" = "-f" ]; then
  	force_build=true   
    fi
    if [ "$input" = "-v" ]; then
  	verbose=true   
    fi
done

# Baseline Dir
AnyFeature_PATH=$(realpath "$0")
AnyFeature_DIR=$(dirname "$AnyFeature_PATH")

## Compile DBoW2 
source_folder="${AnyFeature_DIR}/Thirdparty/DBoW2"     
build_folder="$source_folder/build"
bin_folder="$source_folder/bin"
lib_folder="$source_folder/lib"

if [ "$force_build" = true ]; then
	delete_if_exists ${source_folder}
fi

if [ "$verbose" = true ]; then
	echo "[AnyFeature-VSLAM][build.sh] Compile DBoW2 ... "   
	cmake -G Ninja -B $build_folder -S $source_folder -DCMAKE_PREFIX_PATH=$source_folder -DCMAKE_INSTALL_PREFIX=$source_folder 
	cmake --build $build_folder --config Release 
	ninja install -C $build_folder
else
        echo "[AnyFeature-VSLAM][build.sh] Compile DBoW2 (output disabled) ... "   
	cmake -G Ninja -B $build_folder -S $source_folder -DCMAKE_PREFIX_PATH=$source_folder -DCMAKE_INSTALL_PREFIX=$source_folder > /dev/null 2>&1
	cmake --build $build_folder --config Release > /dev/null 2>&1
	ninja install -C $build_folder > /dev/null 2>&1
fi

## Compile AnyFeature-VSLAM   
source_folder="${AnyFeature_DIR}"     
build_folder="$source_folder/build"
bin_folder="$source_folder/bin"
lib_folder="$source_folder/lib"

if [ "$force_build" = true ]; then
	delete_if_exists ${source_folder}
fi

if [ "$verbose" = true ]; then
        echo "[AnyFeature-VSLAM][build.sh] Compile AnyFeature-VSLAM ... "  
	cmake -G Ninja -B $build_folder -S $source_folder -DCMAKE_PREFIX_PATH=$source_folder -DCMAKE_INSTALL_PREFIX=$source_folder 
	cmake --build $build_folder --config Release 
else    
	echo "[AnyFeature-VSLAM][build.sh] Compile AnyFeature-VSLAM (output disabled) ..."   
	cmake -G Ninja -B $build_folder -S $source_folder -DCMAKE_PREFIX_PATH=$source_folder -DCMAKE_INSTALL_PREFIX=$source_folder > /dev/null 2>&1
	cmake --build $build_folder --config Release > /dev/null 2>&1
fi

## Download vocabulary
vocabulary_folder="${AnyFeature_DIR}/anyfeature_vocabulary"
if [ ! -d "${vocabulary_folder}" ]; then
	mkdir $vocabulary_folder
	python "${AnyFeature_DIR}/download_vocabulary.py"
fi



