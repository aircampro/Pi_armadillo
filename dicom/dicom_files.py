# example using python to process DICOM images which are used in the medical industry
#

# pip install pydicom
# $ pip install matplotlib
# $ pip install -U python-gdcm
# $ pip install pylibjpeg

import pydicom
import matplotlib.pyplot as plt

# show data
file = pydicom.dcmread('sample/my_dicom_file.dcm')        # path to the file
print(file)
# Dataset.file_meta -------------------------------
#(0002, 0000) File Meta Information Group Length  UL: 174
#(0002, 0001) File Meta Information Version       OB: b'\x00\x01'
#(0002, 0002) Media Storage SOP Class UID         UI: Computed Radiography Image Storage

print(file.PatientName)           # patient ID specified by keyword
print(file.[(0x0010, 0x0010)])    # specified by tag https://www.dicomlibrary.com/dicom/dicom-tags/ here it is 	Patient's Name
print(file.[(0x0010, 0x0040)])    # specified by tag https://www.dicomlibrary.com/dicom/dicom-tags/ here it is 	Patient's Sex
print(file.[(0x0010, 0x0030)])    # specified by tag https://www.dicomlibrary.com/dicom/dicom-tags/ here it is 	Patient's Birth Date
print(file.[(0x0010, 0x1040)])    # specified by tag https://www.dicomlibrary.com/dicom/dicom-tags/ here it is 	Patient's Address
print(file.[(0x0018, 0x1182)])    # specified by tag https://www.dicomlibrary.com/dicom/dicom-tags/ here it is 	Focal Distance
print(file.[(0x0028, 0x9507)])    # specified by tag https://www.dicomlibrary.com/dicom/dicom-tags/ here it is 	LUT correction
print(file.[(0x5600, 0x0020)])    # specified by tag https://www.dicomlibrary.com/dicom/dicom-tags/ here it is 	Spectroscopy Data
print(file.[(0x0008, 0x1070)])    # specified by tag https://www.dicomlibrary.com/dicom/dicom-tags/ here it is 	Operators Name

# show image
file = pydicom.dcmread('sample/my_dicom_file.dcm')
img = file.pixel_array
plt.imshow(img)
plt.show()

# If there is data such as WindowCenter or WindowWidth in the DICOM data, it is better to perform this process.
file = pydicom.dcmread('filepath')
wc = file.WindowCenter
ww = file.WindowWidth
img = file.pixel_array

#ウィンドウ処理
window_max = wc + ww /2
window_min = wc - ww /2
img = 255*(img-window_min)/(window_max - window_min)
img[img > 255] = 255
img[img < 0] = 0
plt.imshow(img)
#plt.show(img)
plt.show()