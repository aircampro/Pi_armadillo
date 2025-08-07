# https://pypi.org/project/msoffcrypto-tool/#supported-encryption-methods
# sha256 PASSWORD of word docx
#
from msoffcrypto.format.ooxml import OOXMLFile

PASS2USE="urPassw0rd"
URDOC="secret.docx"
OUTDOC="encrypted.docx"

plain = open(URDOC, "rb")
file = OOXMLFile(plain)

with open(OUTDOC, "wb") as f:
    file.encrypt(PASS2USE, f)

plain.close()