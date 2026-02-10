import cv2 as cv
import numpy as np

# =========================
# CONFIG
# =========================
IMAGE_PATH = "tablero.png"
TEMPLATE_PATH = "template_almacen.png"

ROIS = [
    (120, 350, 260, 200),  # izquierda
    (430, 350, 260, 200),  # centro
    (740, 350, 260, 200),  # derecha
]

METHOD = cv.TM_CCOEFF_NORMED
MIN_SCORE = 0.25
USE_EDGES = False

# =========================
# PREPROCESADO
# =========================
def preprocess(bgr, use_edges):
    gray = cv.cvtColor(bgr, cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray, (3, 3), 0)

    if use_edges:
        return cv.Canny(gray, 60, 160)

    return cv.normalize(gray, None, 0, 255, cv.NORM_MINMAX)

# =========================
# MATCH EN ROI
# =========================
def match_in_roi(img_proc, tpl_proc, roi):
    x, y, w, h = roi
    roi_proc = img_proc[y:y+h, x:x+w]

    th, tw = tpl_proc.shape[:2]
    res = cv.matchTemplate(roi_proc, tpl_proc, METHOD)
    _, max_val, _, max_loc = cv.minMaxLoc(res)

    center_abs = (
        x + max_loc[0] + tw // 2,
        y + max_loc[1] + th // 2
    )

    return {
        "center_abs": center_abs,
        "score": float(max_val)
    }

# =========================
# MAIN
# =========================
def detect_almacenes():
    img = cv.imread(IMAGE_PATH)
    tpl = cv.imread(TEMPLATE_PATH)

    img_proc = preprocess(img, USE_EDGES)
    tpl_proc = preprocess(tpl, USE_EDGES)

    centers = []

    for roi in ROIS:
        result = match_in_roi(img_proc, tpl_proc, roi)

        if result["score"] < MIN_SCORE:
            print(f" Score bajo en ROI {roi}: {result['score']:.3f}")

        centers.append(result["center_abs"])

    return centers


if __name__ == "__main__":
    almacenes = detect_almacenes()
    print("Centros de los almacenes:", almacenes)

