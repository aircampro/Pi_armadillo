class CRC16:
    ARC          = lambda: CRC16(0x8005, 0x0000,  True,  True, 0x0000)
    AUG_CCITT    = lambda: CRC16(0x1021, 0x1d0f, False, False, 0x0000)
    BUYPASS      = lambda: CRC16(0x8005, 0x0000, False, False, 0x0000)
    CCITT_FALSE  = lambda: CRC16(0x1021, 0xffff, False, False, 0x0000)
    CDMA2000     = lambda: CRC16(0xc867, 0xffff, False, False, 0x0000)
    CMS          = lambda: CRC16(0x8005, 0xffff, False, False, 0x0000)
    DDS_110      = lambda: CRC16(0x8005, 0x800d, False, False, 0x0000)
    DECT_R       = lambda: CRC16(0x0589, 0x0000, False, False, 0x0001)
    DECT_X       = lambda: CRC16(0x0589, 0x0000, False, False, 0x0000)
    DNP          = lambda: CRC16(0x3d65, 0x0000,  True,  True, 0xffff)
    EN_13757     = lambda: CRC16(0x3d65, 0x0000, False, False, 0xffff)
    GENIBUS      = lambda: CRC16(0x1021, 0xffff, False, False, 0xffff)
    GSM          = lambda: CRC16(0x1021, 0x0000, False, False, 0xffff)
    LJ1200       = lambda: CRC16(0x6f63, 0x0000, False, False, 0x0000)
    MAXIM        = lambda: CRC16(0x8005, 0x0000,  True,  True, 0xffff)
    MCRF4XX      = lambda: CRC16(0x1021, 0xffff,  True,  True, 0x0000)
    OPENSAFETY_A = lambda: CRC16(0x5935, 0x0000, False, False, 0x0000)
    OPENSAFETY_B = lambda: CRC16(0x755b, 0x0000, False, False, 0x0000)
    PROFIBUS     = lambda: CRC16(0x1dcf, 0xffff, False, False, 0xffff)
    RIELLO       = lambda: CRC16(0x1021, 0x554d,  True,  True, 0x0000)
    T10_DIF      = lambda: CRC16(0x8bb7, 0x0000, False, False, 0x0000)
    TELEDISK     = lambda: CRC16(0xa097, 0x0000, False, False, 0x0000)
    TMS37157     = lambda: CRC16(0x1021, 0x3791,  True,  True, 0x0000)
    USB          = lambda: CRC16(0x8005, 0xffff,  True,  True, 0xffff)
    CRC_A        = lambda: CRC16(0x1021, 0x6363,  True,  True, 0x0000)
    KERMIT       = lambda: CRC16(0x1021, 0x0000,  True,  True, 0x0000)
    MODBUS       = lambda: CRC16(0x8005, 0xffff,  True,  True, 0x0000)
    NRSC_5       = lambda: CRC16(0x080b, 0xffff,  True,  True, 0x0000)
    X_25         = lambda: CRC16(0x1021, 0xffff,  True,  True, 0xffff)
    XMODEM       = lambda: CRC16(0x1021, 0x0000, False, False, 0x0000)

    def __init__(self, poly, init, refin, refout, xorout):
        self.poly = poly
        self.init = init
        self.refin = refin
        self.refout = refout
        self.xorout = xorout
        if self.refin:
            self.init = self.__reflect(init, 16)

    def __reflect(self, x, bits):
        r = 0
        for i in range(bits):
            r = (r << 1) | ((x >> i) & 1)
        return r

    def update(self, data):
        for x in data:
            self.init ^= (self.__reflect(x, 8) if self.refin else x) << 8
            for _ in range(8):
                if self.init & 0x8000:
                    self.init = ((self.init << 1) ^ self.poly) & 0xffff
                else:
                    self.init = (self.init << 1) & 0xffff

    def digest(self, data=b''):
        self.update(data)
        x = self.init
        if self.refout:
            x = self.__reflect(x, 16)
        return x ^ self.xorout