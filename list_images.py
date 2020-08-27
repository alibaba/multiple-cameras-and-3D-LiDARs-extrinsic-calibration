import sys
import os
from argparse import ArgumentParser


def common_arg_parser():

    argparser = ArgumentParser(
        description="Generate list file of images in a directory."
    )
    argparser.add_argument(
        "-i",
        "--input_images_folder",
        metavar="input_images_folder",
        default="",
        dest="input_images_folder",
        help="images folder where contain images",
    )

    argparser.add_argument(
        "-o",
        "--output_folder",
        metavar="output_folder",
        default="",
        dest="output_folder",
        help="output folder for image list file",
    )

    argparser.add_argument(
        "-r",
        "--rewrite_image_name",
        action="store_true",
        dest="rewrite_img_name",
        help="rewrite image name or not",
    )

    return argparser


def sort_by_suffix(img_name_1):
    ext_1 = [e for e in img_name_1.split("_") if e.endswith(".png")][0]
    timestamp_1 = ext_1[:-4]
    if timestamp_1.isdigit():
        return int(timestamp_1)
    return int(timestamp_1[1:])


if __name__ == "__main__":

    parser = common_arg_parser()
    args = parser.parse_args()

    input_folder = args.input_images_folder
    output_file = os.path.join(args.output_folder, "image_list.txt")
    rewrite_flag = args.rewrite_img_name

    files = os.listdir(input_folder)
    image_files = [
        f
        for f in files
        if os.path.isfile(os.path.join(input_folder, f)) and f.endswith(".png")
    ]

    sorted_image_files = sorted(image_files, key=sort_by_suffix)
    print(sorted_image_files)

    img_idx = 0
    with open(output_file, "w") as output:
        for s in sorted_image_files:
            new_img_name = "image_{}.png".format(img_idx)
            print(new_img_name)
            img_idx += 1

            old_img_abs_path = os.path.join(input_folder, s)
            new_img_abs_path = os.path.join(input_folder, new_img_name)
            if rewrite_flag:
                os.rename(old_img_abs_path, new_img_abs_path)
                output.write(new_img_name)
            else:
                output.write(s)

            output.write("\n")
