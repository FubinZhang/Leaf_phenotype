import cv2
import numpy as np

class Leaf:
    def __init__(self):
        self.errors = ""
        self.src_gray = None
        self.src_otsu = None
        self.src_hsv = None
        self.leaf_length = 0.0
        self.leaf_width = 0.0
        self.length_width_ratio = 0.0
        self.leaf_perimeter = 0.0
        self.leaf_area = 0.0
        self.rectangularity = 0.0
        self.densification = 0.0
        self.circularity = 0.0
        self.sphericity = 0.0
        self.boundary_energy = 0.0
        self.leaf_color = ""

class Scan2:
    def __init__(self):
        pass

    @staticmethod
    def set_pad_black(image, d):
        rows, cols = image.shape[:2]
        for i in range(rows):
            for j in range(cols):
                if i < d or j < d or rows - i < d or cols - j < d:
                    image[i, j] = 0

    @staticmethod
    def cmp(A, B):
        return len(A) > len(B)

    def find_longest_line(self, lines):
        lines.sort(key=len, reverse=True)
        lines = lines[:1]

    @staticmethod
    def change_contours(linein, center, rangle):
        pi = np.pi
        lineout = []
        for point in linein:
            x1 = (point[0][0] - center[0]) * np.cos(rangle / 180 * pi) - (point[0][1] - center[1]) * np.sin(rangle / 180 * pi) + center[0]
            y1 = (point[0][0] - center[0]) * np.sin((rangle / 180 * pi)) + (point[0][1] - center[1]) * np.cos(rangle / 180 * pi) + center[1]
            lineout.append((int(x1), int(y1)))
        return lineout

    def draw_contour(self, image, Line):
        for pt in Line:
            image[pt[1], pt[0]] = 255

    @staticmethod
    def arc_circularity(line, center):
        avedistance = 0
        vardistance = 0

        for point in line:
            avedistance += np.sqrt((point[0][0] - center[0]) ** 2 + (point[0][1] - center[1]) ** 2)

        avedistance = avedistance / len(line)

        for point in line:
            vardistance += (np.sqrt((point[0][0] - center[0]) ** 2 + (point[0][1] - center[1]) ** 2) - avedistance) ** 2

        vardistance = vardistance / len(line)
        circularity = avedistance / vardistance

        return circularity

    @staticmethod
    def arc_sphericity(line, center, rec, reduce):
        maxcir_center, outcir_radius = cv2.minEnclosingCircle(np.array(line, dtype=np.float32))
        dist, maxdist = 0, 0
        mincir_center = (0, 0)
        incir_radius = 0

        narrow_line = []
        for point in line:
            narrow_point = (center[0] - int((center[0] - point[0]) / reduce), center[1] - int((center[1] - point[1]) / reduce))
            narrow_line.append(narrow_point)

        narrow_line = np.array(narrow_line, dtype=np.float32)
        for i in range(int(center[0] - rec[2] / reduce), int(center[0] + rec[2] / reduce)):
            for j in range(int(center[1] - rec[3] / reduce), int(center[1] + rec[3] / reduce)):
                dist = cv2.pointPolygonTest(narrow_line, (i, j), True)
                if dist > maxdist:
                    maxdist = dist
                    mincir_center = (i, j)

        mincir_center = (center[0] - int((center[0] - mincir_center[0]) * reduce), center[1] - int((center[1] - mincir_center[1]) * reduce))
        incir_radius = cv2.pointPolygonTest(np.array(line, dtype=np.float32), mincir_center, True)
        sphericity = incir_radius / outcir_radius

        return sphericity

    @staticmethod
    def arc_boundary_energy(line, inarray):
        E = 0
        halfstep2 = 30
        line = np.array(line)  # Assuming `line` is a list of tuples (x, y)
        three = []
        curvature = []
        for i in range(len(line)):
            last = (i - halfstep2) % len(line)
            next = (i + halfstep2) % len(line)
            a, b, c = line[last], line[i], line[next]
            three.append([a, b, c])
            if a[0] == b[0] == c[0] or a[1] == b[1] == c[1]:
                curvature.append(0)
                three = []
            else:
                dis1 = np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
                dis2 = np.sqrt((a[0] - c[0]) ** 2 + (a[1] - c[1]) ** 2)
                dis3 = np.sqrt((b[0] - c[0]) ** 2 + (b[1] - c[1]) ** 2)
                dis = dis1 ** 2 + dis3 ** 2 - dis2 ** 2
                cosA = dis / (2 * dis1 * dis3)
                sinA = np.sqrt(1 - cosA ** 2)
                radius = 0.5 * dis2 / sinA
                curvature.append(1 / radius)
                three = []
                if radius < inarray.shape[1]:  # Assuming `inarray` is a NumPy array
                    E += curvature[-1] ** 2

        E /= len(line)
        return E

    @staticmethod
    def count_color(img, left, right):
        x1, x2, y1, y2 = left[1], right[1], left[0], right[0]
        greens, yellows, light_yellows, light_greens = 0, 0, 0, 0
        for i in range(x1, x2 + 1):
            for j in range(y1, y2 + 1):
                H = img[i, j, 0]
                if 25 <= H <= 32:
                    light_yellows += 1
                elif 19 <= H <= 24:
                    yellows += 1
                elif 35 <= H <= 37:
                    light_greens += 1
                elif 37 <= H <= 61:
                    greens += 1

        all_colors = greens + yellows + light_greens + light_yellows
        GSize = greens / all_colors
        YSize = yellows / all_colors
        LGSize = light_greens / all_colors
        LYSize = light_yellows / all_colors
        MAX = max(GSize, YSize, LGSize, LYSize)
        if MAX == GSize:
            color = "Green"
        elif MAX == YSize:
            color = "Yellow"
        elif MAX == LGSize:
            color = "Light Green"
        else:
            color = "Light Yellow"

        return color

    def process_src(self, path):
        set_leaf = Leaf()
        src = cv2.imread(path, cv2.IMREAD_COLOR)
        if src is None:
            set_leaf.errors = "未能读取图片源"
            return set_leaf

        kc = 29.7 / src.shape[0]
        set_leaf.src_gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        _, set_leaf.src_otsu = cv2.threshold(set_leaf.src_gray, 60, 255, cv2.THRESH_OTSU)
        set_leaf.src_hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)

        for i in range(set_leaf.src_otsu.shape[0]):
            for j in range(set_leaf.src_otsu.shape[1]):
                val = set_leaf.src_otsu[i, j]
                if val == 0:
                    set_leaf.src_otsu[i, j] = 255
                else:
                    set_leaf.src_otsu[i, j] = 0
        self.set_pad_black(set_leaf.src_otsu, min(set_leaf.src_otsu.shape[:2]) // 100)

        contours, _ = cv2.findContours(set_leaf.src_otsu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = sorted(contours, key=len, reverse=True)
        self.find_longest_line(contours)
        test = np.zeros(set_leaf.src_gray.shape, dtype=np.uint8)
        cv2.drawContours(test, contours, 0, 255, 2)

        moments = cv2.moments(contours[0])
        centroid = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))

        box = cv2.fitEllipse(contours[0])
        line_transform = self.change_contours(contours[0], box[0], 90 - box[2])
        rec = cv2.boundingRect(np.array(line_transform, dtype=np.float32))

        set_leaf.leaf_length = rec[2] * kc
        set_leaf.leaf_width = rec[3] * kc
        set_leaf.length_width_ratio = set_leaf.leaf_length / set_leaf.leaf_width
        set_leaf.leaf_perimeter = cv2.arcLength(contours[0], True) * kc
        set_leaf.leaf_area = moments["m00"] * kc * kc
        set_leaf.rectangularity = moments["m00"] / (rec[2] * rec[3])
        set_leaf.densification = (set_leaf.leaf_perimeter ** 2) / set_leaf.leaf_area
        set_leaf.circularity = self.arc_circularity(contours[0], centroid)
        set_leaf.sphericity = self.arc_sphericity(line_transform, box[0], rec, 10)
        set_leaf.boundary_energy = self.arc_boundary_energy(line_transform, src)
        set_leaf.leaf_color = self.count_color(set_leaf.src_hsv, rec[:2], (rec[0] + rec[2], rec[1] + rec[3]))

        return set_leaf

if __name__ == "__main__":
    path = "leaf.png"
    B = Scan2()
    A = B.process_src(path)
    if A.errors != "":
        print(A.errors)
    else:
        print("ok")

    print(A.leaf_length)
    print(A.leaf_width)
    print(A.length_width_ratio)
    print(A.leaf_perimeter)
    print(A.leaf_area)
    print(A.rectangularity)
    print(A.densification)
    print(A.circularity)
    print(A.sphericity)
    print(A.boundary_energy)
    print(A.leaf_color)
