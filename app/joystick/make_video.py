#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import subprocess


def filename_cmp(filename):
    # for rk_anim
    return float(os.path.basename(filename)[4:-4])


def file_sort(dirname):
    files = glob.glob(dirname + '/*.png')
    if files[0].startswith(os.path.join(dirname, "capture")):
        print("not renamed")
        return False
    files.sort(key=filename_cmp)
    print("renaming... ", )
    for i, src in enumerate(files):
        # print(src, "->", dirname + "/capture%05d.png" % i)
        os.rename(src, dirname + "/capture%05d.png" % i)
    print("done.")
    return True


def make_video(dirname, framerate):
    file_sort(dirname)
    # for rk_anim
    # args = ["avconv", "-r", "33", "-i", dirname + "/capture%05d.png", "-q:v", "1", "video.avi"]

    # args = ["avconv", "-r", "90", "-i", dirname + "/capture%05d.png", "-q:v", "1", "video.avi"]
    # args = ["avconv", "-r", "100", "-i", dirname + "/capture%05d.png", "-q:v", "1", "video.avi"]
    # args = ["avconv", "-r", "300", "-i", dirname + "/capture%05d.png", "-q:v", "1", "video.avi"]
    args = ["ffmpeg", "-framerate", framerate, "-i", dirname + "/capture%05d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p", "video.mp4"]
    # print(" ".join(args))
    subprocess.call(" ".join(args), shell=True)


def mkdir(dirname):
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    return dirname


def append(dir1, dir2, dir3, out):
    mkdir(out)
    files = glob.glob(os.path.join(dir1, "capture*.png"))
    for i in range(len(files)):
        file1 = os.path.join(dir1, "capture%05d.png" % i)
        file2 = os.path.join(dir2, "capture%05d.png" % i)
        file3 = os.path.join(dir3, "capture%05d.png" % i)
        fileout = os.path.join(out, "movie%05d.png" % i)
        # print(file1, file2, file3, fileout)
        args = ["convert", "+append", file1, file2, file3, fileout]
        subprocess.call(" ".join(args), shell=True)


def make_gif(dirname):
    dir1 = os.path.join(dirname, "cap")
    # dir2 = os.path.join(dirname, "X")
    # dir3 = os.path.join(dirname, "P")
    dir2 = os.path.join(dirname, "state")
    dir3 = os.path.join(dirname, "param")
    out = os.path.join(dirname, "movie")
    ret1 = file_sort(dir1)
    ret2 = file_sort(dir2)
    ret3 = file_sort(dir3)
    if ret1 or ret2 or ret3:
        append(dir3, dir2, dir1, out)
    args = ["avconv", "-r", "20", "-i", out + "/movie%05d.png", "-q:v", "1", out + "/movie.avi"]
    # args = ["convert", "-delay", "33", "-loop", "0", out + "/movie*.png", out + "/movie.gif"]
    subprocess.call(" ".join(args), shell=True)


def usage(prog):
    print("%s directory <avi/gif>" % prog)


def main():
    import sys

    if len(sys.argv) <= 2:
        usage(sys.argv[0])
        exit()
    else:
        if sys.argv[2] == "avi":
            make_video(sys.argv[1], sys.argv[3])
        elif sys.argv[2] == "gif":
            make_gif(sys.argv[1])
        else:
            usage(sys.argv[0])
            exit()


if __name__ == '__main__':
    main()
