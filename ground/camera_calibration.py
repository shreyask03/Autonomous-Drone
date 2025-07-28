import cv2
import numpy as np
import glob

# Settings — adjust as needed
chessboard_size = (8, 5)  # number of inner corners per row and column of a 9x6 chessboard
square_size = 1.0         # arbitrary unit (e.g., 1.0 for normalized units, or cm)

# Termination criteria for cornerSubPix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare 3D object points, like (0,0,0), (1,0,0), ..., (8,5,0)
objp = np.zeros((chessboard_size[1]*chessboard_size[0], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store 3D points and 2D image points
objpoints = []  # 3D points
imgpoints = []  # 2D points

images = glob.glob('calib_images/*.jpg')  # or .png etc.

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)

        # Refine corner locations
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and show
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Perform calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# Print results
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs.ravel())
print("Reprojection Error:", ret)

# Save calibration for later
np.savez("calibration_data.npz", 
         camera_matrix=camera_matrix, 
         dist_coeffs=dist_coeffs, 
         rvecs=rvecs, 
         tvecs=tvecs)
