#!/usr/bin/env python3
import cv2
import numpy as np
import sys

def sfm_from_video(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Cannot open video {video_path}")
        return None
    
    # Generic calibration matrix (assuming 1920x1080, adjust focal length as needed)
    K = np.array([[1000, 0, 960],
                  [0, 1000, 540],
                  [0, 0, 1]], dtype=np.float32)
    
    sift = cv2.SIFT_create()
    matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    
    ret, prev_frame = cap.read()
    if not ret:
        return None
    
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    prev_kp, prev_desc = sift.detectAndCompute(prev_gray, None)
    
    points_3d = []
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, desc = sift.detectAndCompute(gray, None)
        
        if desc is None or prev_desc is None:
            prev_gray, prev_kp, prev_desc = gray, kp, desc
            continue
        
        matches = matcher.knnMatch(prev_desc, desc, k=2)
        good = [m for m, n in matches if m.distance < 0.7 * n.distance]
        
        if len(good) < 8:
            prev_gray, prev_kp, prev_desc = gray, kp, desc
            continue
        
        pts1 = np.float32([prev_kp[m.queryIdx].pt for m in good])
        pts2 = np.float32([kp[m.trainIdx].pt for m in good])
        
        E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        _, R, t, mask = cv2.recoverPose(E, pts1, pts2, K, mask=mask)
        
        # Triangulate points
        P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1))))
        P2 = K @ np.hstack((R, t))
        
        pts1_norm = pts1[mask.ravel() == 1]
        pts2_norm = pts2[mask.ravel() == 1]
        
        points_4d = cv2.triangulatePoints(P1, P2, pts1_norm.T, pts2_norm.T)
        points_3d_frame = (points_4d[:3] / points_4d[3]).T
        
        points_3d.extend(points_3d_frame)
        
        prev_gray, prev_kp, prev_desc = gray, kp, desc
    
    cap.release()
    return np.array(points_3d)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python sfm.py <video_path>")
        sys.exit(1)
    
    points = sfm_from_video(sys.argv[1])
    
    if points is not None and len(points) > 0:
        print(f"Detected {len(points)} 3D points")
        np.save("sfm_points.npy", points)
        print("Saved to sfm_points.npy")
    else:
        print("No points detected")
