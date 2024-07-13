import os
import sys
import subprocess
import argparse

SCRIPT_LABEL = '[createVocabulary.py] '
HOME_PATH = f"{os.path.expanduser('~')}"
ANYFEATURE_VSLAM_FOLDER = os.path.dirname(os.path.abspath(__file__))
CONDA_PREFIX = os.getenv('CONDA_PREFIX')

FEATURES =  {
    0: ("Orb32", True),
    1: ("Akaze61", True),
    2: ("Brisk48", True),
    3: ("Surf64", False),
    4: ("Kaze64", False),
    5: ("Sift128", False),
    6: ("R2d2", False),
    7: ("AnyFeatBin", False),
    8: ("AnyFeatNonBin", False),
}
def main():

    print(f"\n{SCRIPT_LABEL}Executing: {__file__}")

    # Parse inputs
    parser = argparse.ArgumentParser(description='createVocabulary.py')

    parser.add_argument("--rgb_folder", type = str, default = os.path.join(HOME_PATH,'BOVISA','2008-09-01','rgb'),
                        const = os.path.join(HOME_PATH,'BOVISA','2008-09-01','rgb'),
                        nargs = '?',
                        help = f"Path to the folder containing the rgb images. Default: {os.path.join(HOME_PATH,'BOVISA','2008-09-01','rgb')}")
    parser.add_argument("--rgb_txt", type = str, default = os.path.join(ANYFEATURE_VSLAM_FOLDER,'docs','bovisa.txt'),
                    const = os.path.join(ANYFEATURE_VSLAM_FOLDER,'docs','bovisa.txt'),
                    nargs = '?',
                    help = f"Txt file containing the list of rgb images. Default: {os.path.join(ANYFEATURE_VSLAM_FOLDER,'docs','bovisa.txt')}")
    parser.add_argument("--rgb_frequency", type=int, default=6, const=6, nargs='?', help="Value to sample every i-th rgb image. Default: 6")

    parser.add_argument('--branching_factor', nargs='?', type=int,
                        const=10, default=10, help="Branching factor. Default: 10")
    parser.add_argument('--depth_levels', nargs='?', type=int,
                    const=6, default=6, help="Depth Levels. Default: 6")

    parser.add_argument("--feature_id", type=int, required=True, help="Feature id. Example: ?????????????????")

    parser.add_argument("--saveVocabulary_path", type = str, default = os.path.join(CONDA_PREFIX,'share','Vocabulary'),
                        const = os.path.join(CONDA_PREFIX,'shared','Vocabulary'),
                        nargs = '?',
                        help = f"??????????????. Default: { os.path.join(CONDA_PREFIX,'share','Vocabulary')}")
    parser.add_argument("--featSettYamlFile", type = str)
        
    args = parser.parse_args()

    if(args.feature_id == 7):
        print("\nYou are using your own BINARY FEATURE. Make sure you have:")
        print("    - Updated members and methods of class Feature_AnyFeatBin in:")
        print(f"            {os.path.join(ANYFEATURE_VSLAM_FOLDER,'include','Feature_AnyFeatBin.h')}")
        print(f"            {os.path.join(ANYFEATURE_VSLAM_FOLDER,'src','Feature_AnyFeatBin.cpp')}")
        print("    - Updated the Descriptor length L (in bytes) in:")
        print(f"            {os.path.join(ANYFEATURE_VSLAM_FOLDER,'Thirdparty','DBoW2','include','DBoW2','FAnyFeatBin.h')}")
        print("    - Execute: ")
        print(f"            python {os.path.join(ANYFEATURE_VSLAM_FOLDER,'build.py')}\n")

    if(args.feature_id == 8):
        print("\nYou are using your own NON BINARY FEATURE. Make sure you have:")
        print("    - Updated members and methods of class Feature_AnyFeatNonBin in:")
        print(f"            {os.path.join(ANYFEATURE_VSLAM_FOLDER,'include','Feature_AnyFeatNonBin.h')}")
        print(f"            {os.path.join(ANYFEATURE_VSLAM_FOLDER,'src','Feature_AnyFeatNonBin.cpp')}")
        print("    - Updated the Descriptor length L (number of floats) in:")
        print(f"            {os.path.join(ANYFEATURE_VSLAM_FOLDER,'Thirdparty','DBoW2','include','DBoW2','FAnyFeatNonBin.h')}\n")
        print("    - Execute: ")
        print(f"            python {os.path.join(ANYFEATURE_VSLAM_FOLDER,'build.py')}\n")

    # Download dataset
    if(not os.path.isdir(args.rgb_folder)):
        print("Default training dataset BOVISA 2008-09-01 is not available.")
        response = input("Do you want to download it? (yes/no): ").strip().lower()
        if(response == 'yes'):
            print("Downloading BOVISA 2008-09-01...")
            sys.exit(1)
        else:
            print(f"{SCRIPT_LABEL}Exit")
            sys.exit(1)

    # Create rgb.txt
    if(not os.path.isfile(args.rgb_txt)):
        image_extensions = {'.jpg', '.jpeg', '.png', '.gif', '.bmp', '.tiff'}
        rgb_list = [f for f in os.listdir(args.rgb_folder) if os.path.splitext(f)[1].lower() in image_extensions]
        rgb_list = rgb_list[::args.rgb_frequency]

        with open(args.rgb_txt, 'w') as file:
            for rgb in rgb_list:
                file.write(f"/{rgb}\n")

    # Create Vocabulary
    featureName, isBinary = FEATURES.get(args.feature_id,("Unknown", False))
    createVocabulary_exec = os.path.join('./bin','createVocabulary')
    subprocess.run([createVocabulary_exec,
                    str(args.feature_id), featureName, str(int(isBinary)), # Feature specifications
                    args.rgb_folder, args.rgb_txt, args.saveVocabulary_path, # Dataset specifications
                    str(args.branching_factor),str(args.depth_levels),
                    str(args.featSettYamlFile)])

if __name__ == "__main__":
    main()                   
