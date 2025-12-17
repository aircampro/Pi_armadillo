#
# simple call to this class to translate japanese manga https://github.com/kha-white/manga-ocr
#
#
import sys
from PIL import Image
from PIL import UnidentifiedImageError
from manga_ocr import MangaOcr

def do_ocr(img_or_path):
    mocr = MangaOcr()
    img = Image.open(img_or_path)
    text = mocr(img)
    print(text)

if __name__ == '__main__':

    if len(sys.argv) >= 1:
        filen = str(sys.argv[1])
    else:
        print("please pass file name first")
    do_ocr(filen)



