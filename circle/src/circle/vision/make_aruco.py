import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # 4x4 크기의 마커 50개
for i in range(5):  # 5개의 마커 생성
    marker_img = aruco.drawMarker(aruco_dict, i, 700)  # 700x700 크기의 마커 생성
    cv2.imwrite(f"aruco_marker_{i}.png", marker_img)

print("Aruco 마커 생성 완료!")
