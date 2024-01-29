import cv2
import numpy as np
import glob

# チェスボードの内部コーナーの数を設定
rows = 7
cols = 10
square_size = 17.3
pattern_size = (cols, rows)
pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size

# キャリブレーション画像の読み込み
images = glob.glob('calib/*.jpeg')  # 画像のパスを適切に設定

obj_points = []
img_points = []
for fname in images:
    frame = cv2.imread(fname)
    height = frame.shape[0]
    width = frame.shape[1]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corner = cv2.findChessboardCorners(gray, pattern_size)
    if found:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(gray, corner, (5,5), (-1,-1), term)
        #cv2.drawChessboardCorners(frame, pattern_size, corner, found)
        #cv2.imshow('found corners', frame)
        #cv2.waitKey(1)
        img_points.append(corner.reshape(-1, 2))
        obj_points.append(pattern_points)
    else:
    	print("Error: ", fname)

#cv2.destroyAllWindows()

rms, K, d, r, t = cv2.calibrateCamera(obj_points, img_points, (width, height), None, None)
print("RMS = ", rms)
print("K = \n", K)
print("d = ", d.ravel())
np.savetxt("K.csv", K, delimiter =',',fmt="%0.14f")
np.savetxt("d.csv", d, delimiter =',',fmt="%0.14f")

