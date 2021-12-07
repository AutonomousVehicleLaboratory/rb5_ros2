import sys
import os
from os import path as osp
import cv2
import yaml
import numpy as np
from timeit import default_timer as timer


def read_image_from_dir(img_dir):
    img_list = []

    files = os.listdir(img_dir)

    for file in files:
        if file.endswith('.png') or file.endswith('.jpg'):
            img_path = osp.join(img_dir, file)
            img = cv2.imread(img_path)
            img_list.append(img)

    return img_list


def parse_calibration_yaml(yaml_path):
    calibration_dict = {}
    with open(yaml_path) as yaml_fp:
        raw_data = yaml.safe_load(yaml_fp)
        
        calibration_dict["width"] = raw_data["image_width"]
        calibration_dict["height"] = raw_data["image_height"]
        calibration_dict["camera_matrix"]=np.array(
            raw_data["camera_matrix"]["data"]
        ).reshape(
            raw_data["camera_matrix"]["cols"], 
            raw_data["camera_matrix"]["rows"])
        calibration_dict["distortion_coefficients"] = np.array(
            raw_data["distortion_coefficients"]["data"]
        )

        return calibration_dict


def undistort_slow(img_list, mtx, dist, w, h, newcameramtx, roi):
    img_list_undist = []
    x, y, w, h = roi
    
    for img in img_list:
        img_undist = cv2.undistort(img, mtx, dist, None, newcameramtx)
        
        # crop the image    
        img_undist = img_undist[y:y+h, x:x+w]
        
        img_list_undist.append(img_undist)
    

    
    return img_list_undist


def undistort_fast(img_list, mtx, dist, w, h, newcameramtx, roi):
    img_list_undist = []
    x, y, w, h = roi
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)

    for img in img_list:
        img_undist = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        
        # crop the image    
        img_undist = img_undist[y:y+h, x:x+w]
        
        img_list_undist.append(img_undist)
    
    return img_list_undist


def display_image_list(img_list):
    for img in img_list:
        cv2.imshow('image', img)
        cv2.waitKey(0)

def main():
    print(cv2.getBuildInformation())
    img_dir = "/home/henry/Documents/data/000-calibration"
    yaml_path = osp.join(img_dir, "ost.yaml")
    img_list = read_image_from_dir(img_dir)
    calibration_dict = parse_calibration_yaml(yaml_path)
    
    mtx = calibration_dict["camera_matrix"]
    dist = calibration_dict["distortion_coefficients"]
    w = calibration_dict["width"]
    h = calibration_dict["height"]

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    start = timer()
    img_list_undist_slow = undistort_slow(img_list, mtx, dist, w, h, newcameramtx, roi)
    print("slow: ", timer() - start)

    start = timer()
    img_list_undist_fast = undistort_fast(img_list, mtx, dist, w, h, newcameramtx, roi)
    print("fast: ", timer() - start)

    display_image_list(img_list_undist_fast)


main()