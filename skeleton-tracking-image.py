#!/usr/bin/env python3
import util as cm
import cv2
import argparse
import os
import platform
from skeletontracker import skeletontracker

parser = argparse.ArgumentParser(description="Perform keypoing estimation on an image")
parser.add_argument(
    "-c",
    "--confidence_threshold",
    type=float,
    default=0.5,
    help="Minimum confidence (0-1) of displayed joints",
)
parser.add_argument(
    "-v",
    "--verbose",
    action="store_true",
    help="Increase output verbosity by enabling backend logging",
)

parser.add_argument(
    "-o",
    "--output_image",
    type=str,
    help="filename of the output image",
)

parser.add_argument("image", metavar="I", type=str, help="filename of the input image")


# Main content begins
if __name__ == "__main__":
    try:
        # Parse command line arguments and check the essentials
        args = parser.parse_args()

        # Read the image
        img = cv2.imread(args.image)

        # Get the skeleton tracking object
        skeletrack = skeletontracker()

        # Perform skeleton tracking
        skeletons = skeletrack.track_skeletons(img)

        # Render results
        cm.render_result(skeletons, img, args.confidence_threshold)
        print("Detected skeletons: ", len(skeletons))
        if args.verbose:
          print(skeletons)
          
        if args.output_image:
            isSaved = cv2.imwrite(args.output_image, img)
            if isSaved:
                print("The result image is saved in: ", args.output_image)
            else:
                print("Saving the result image failed for the given path: ", args.output_image)


            
    except Exception as ex:
        print("Exception occured: \"{}\"".format(ex))
# Main content ends
