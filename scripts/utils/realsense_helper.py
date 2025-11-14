import numpy as np
import cv2
import pupil_apriltags
def draw_img_detections(image: np.ndarray, tags: list):
    for tag in tags:
        corners = tag.corners
        for i in range(4):
            pt1 = (int(corners[i][0]), int(corners[i][1]))
            pt2 = (int(corners[(i + 1) % 4][0]), int(corners[(i + 1) % 4][1]))
            cv2.line(image, pt1, pt2, (0, 255, 0), 2)
        center = (int(tag.center[0]), int(tag.center[1]))
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        
    if len(tags)==0:
        image = image.copy()
    return image

def get_apriltag_rotation(tag:pupil_apriltags.Detection):
    if hasattr(tag, 'pose_R'):
        R = tag.pose_R
        return R
    else:
        return None

def rotation_matrix_to_euler_angles(R: np.ndarray):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def rotation_matrix_to_quaternion(R: np.ndarray):
    qw = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    return np.array([qx, qy, qz, qw])