#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# ref :- https://github.com/yuki-inaho/purethermal2_pymodule/blob/main/example/main.py
# libusb thermal camera interface
#
import cv2
from purethermal2_pymodule.pt2_api import PyPureThermal2
# write data to logger
from purethermal2_pymodule.utils import get_logger_with_stdout
logger = get_logger_with_stdout("PureThermal2")

SCALE = 4.0
def main():
    pt2_camera = PyPureThermal2()
    while True:
        if pt2_camera.update():
            thermal_image = pt2_camera.thermal_image_colorized.copy()
            thermal_image_scaled = cv2.resize(thermal_image, None, fx=SCALE, fy=SCALE)
            cv2.imshow("thermal", thermal_image_scaled)
            raw_image = pt2_camera.thermal_image()
            raw_image_scaled = cv2.resize(raw_image, None, fx=SCALE, fy=SCALE)
            cv2.imshow("raw", raw_image_scaled)
            #print(f"Cmax: {pt2_camera.thermal_image_cercius.max()}")
            #print(f"Cmin: {pt2_camera.thermal_image_cercius.min()}")
            #print(f"Cmean: {pt2_camera.thermal_image_cercius.mean()}")
            #print(f"Cstd: {pt2_camera.thermal_image_cercius.srd()}")
            logger.info(f"Cmax: {pt2_camera.thermal_image_cercius.max()}")
            logger.info(f"Cmin: {pt2_camera.thermal_image_cercius.min()}")
            logger.info(f"Cmean: {pt2_camera.thermal_image_cercius.mean()}")
            logger.info(f"Cstd: {pt2_camera.thermal_image_cercius.std()}")
        key = cv2.waitKey(10)

        if key & 0xFF in [ord("q"), 27]:
            break
        if key & 0xFF == ord("s"):
            if "thermal_image_scaled" in locals():
                print("saved")
                cv2.imwrite("thermal.png", thermal_image_scaled)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()