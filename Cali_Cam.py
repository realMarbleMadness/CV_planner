import numpy as np
from numpy import linalg as LA
import math
import pdb

import cv2 as cv
import os
import glob

class Cali_Cam:
    def __init__(self, checker_size=24, num_row=7, num_col=10): 
        self.checker_size = checker_size
        self.num_row = num_row
        self.num_col = num_col

    def extract_points(self, img, visualize=False):
        # returns target CB points and extracted CB points

        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # point container
        target_coord = np.zeros ((self.num_col*self.num_row, 3), np.float32)
        count = 0
        for i in range(0, self.num_col):
            for j in range(self.num_row-1, -1, -1):
                target_coord[count, 0] = i
                target_coord[count, 1] = j
                count += 1

        target_coord = target_coord*self.checker_size

        target_points = [] # 3d point in real world space
        img_points = [] # 2d points in image plane.

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        gray = gray[0:gray.shape[0]/2, 0:gray.shape[1]/2]  # for some reason, checkerboard cannot be detected in some pictures, cropping the image to its upper left quarter
        # cv.imshow('gray', gray)
        # cv.waitKey(0)
        # pdb.set_trace()

        ret, corners = cv.findChessboardCorners(gray, (self.num_row, self.num_col), None)
        #pdb.set_trace()
        # If found, add object points, image points (after refining them)
        if ret == True:
            target_points.append(target_coord)
            corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            img_points.append(corners)

            if visualize:
                # Draw and display the corners
                cv.drawChessboardCorners(img, (self.num_row, self.num_col), corners2, ret)
                cv.imshow('img', img)
                cv.waitKey(0)
                cv.destroyAllWindows()

        return target_points, img_points


    def CB_bounds(self, points):
        # unfortunately, returns the bottom left and upper right of the checkerboard
        # pdb.set_trace()

        CB_origin = points[0][0]
        CB_origin = np.squeeze(CB_origin, 0)
        CB_diagonal = points[0][-1]
        CB_diagonal = np.squeeze(CB_diagonal, 0)

        # needs tuning
        x_scale = self.checker_size/((CB_diagonal[0] - CB_origin[0])/(self.num_col - 1))
        y_scale = self.checker_size/((CB_origin[1] - CB_diagonal[1])/(self.num_row - 1))

        # scale = # of mm/pixel
        return CB_origin, CB_diagonal, x_scale, y_scale 


    def camera_params(self, target_points, img_points, img):
        # get intrinsic and extrinsics

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, intrinsic, dist, rotation, translation = cv.calibrateCamera(target_points, img_points, gray.shape[::-1], None, None)
        # cam_rot = cv.Rodrigues(rotation[0])[0]
        # cam_trans = np.matrix(cam_rot).T*np.matrix(translation[0])
        # cam_trans[2] -= 1000
        # cam_trans = np.squeeze(cam_trans, axis=1)
        # H = np.zeros((4,4))
        # H[0:3, 0:3] = cam_rot
        # H[0:3, 3] = cam_trans
        # H[3,3] = 1 

        mat_intrin = np.array([[924.1445, 0,        0],  # from matlab
                               [0,        924.4962, 0], 
                               [636.5713, 364.4404, 1]])
        mat_intrin = np.transpose(mat_intrin)  # representation is different

        # this is much better
        ret, rvecs, tvecs = cv.solvePnP(target_points[0], img_points[0], mat_intrin, None)
        cam_rot = cv.Rodrigues(rvecs)[0]
        cam_trans = np.matrix(cam_rot).T*np.matrix(tvecs)
        cam_trans = np.squeeze(cam_trans, axis=1)
        H = np.zeros((4,4))
        H[0:3, 0:3] = cam_rot
        H[0:3, 3] = cam_trans
        H[3,3] = 1 

        return H, mat_intrin

