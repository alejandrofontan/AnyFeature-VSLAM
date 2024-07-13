import os
import sys
import tarfile

from huggingface_hub import hf_hub_download
import pandas as pd

vocabulary_files = [
"ORBvoc.txt",
"Akaze61_DBoW2_voc.txt",
"Brisk_DBoW2_voc.txt",
"Surf64_DBoW2_voc.txt",
"Sift128_DBoW2_voc.txt",
"Kaze64_DBoW2_voc.txt",
"R2d2_DBoW2_voc.txt"
]

REPO_ID = "fontan/anyfeature_vocabulary"

def main():
      
    current_script_path = __file__
    anyFeature_directory = os.path.dirname(os.path.abspath(current_script_path))
    
    ## Extract Vocabulary
    print("[AnyFeature-VSLAM][download_vocabulary.py] Download vocabulary ... ")   
    vocabulary_folder = os.path.join(anyFeature_directory, 'anyfeature_vocabulary')
    for vocabulary_file in vocabulary_files:
        dataset = pd.read_csv(
            hf_hub_download(repo_id=REPO_ID, filename=vocabulary_file, repo_type="dataset")
        )
        dataset.to_csv(os.path.join(vocabulary_folder, vocabulary_file), sep='\t', index=False)
                           
if __name__ == "__main__":
    main()                
