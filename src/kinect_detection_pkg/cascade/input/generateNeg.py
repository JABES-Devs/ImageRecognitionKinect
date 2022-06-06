import os

def generate_negative_description_file():
    # open the output file for writing. Will overwrite all existing data in there
    with open('neg.txt','w') as f:
        # loop over all the filenames
        for filename in os.listdir('negative'):
            f.write('negative/' + filename + '\n')

if __name__ == '__main__':
    generate_negative_description_file()