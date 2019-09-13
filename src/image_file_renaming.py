import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("dir", help="directory where images are stored", type=str)
args = parser.parse_args()
for filename in os.listdir(args.dir):
    if filename.endswith(".png"):
        src = filename
        dst = "img" + filename
        os.rename(src, dst)

