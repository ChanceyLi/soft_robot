import struct


class FrameData:
    def __init__(self, pixStyle=0x14, width=1, height=1):

        self.pixStyle = pixStyle
        self.frameWidth = width
        self.frameHeight = height

        self.pkgLen = self.frameWidth * self.frameHeight * (pixStyle & 0x0f) + 6

        self.buf = [0 for i in range(self.pkgLen)]
        # 数据头尾定义
        self.buf[0] = 0xff
        self.buf[1] = self.pixStyle
        self.buf[2] = self.frameWidth
        # 补丁
        if width > 255:
            self.buf[1] = 0x40 + (pixStyle & 0x0f)
            self.buf[2] = 128

        self.buf[3] = self.frameHeight
        self.buf[self.pkgLen - 2] = (self.pixStyle + self.frameWidth + self.frameHeight) & 0xff
        self.buf[self.pkgLen - 1] = 0xfe

    def setCrc(self):
        r = 0
        for i in range(1, self.pkgLen - 2):
            r += self.buf[i]
        self.buf[self.pkgLen - 2] = r & 0xff

    def setDataToArray(self, arr):
        alen = len(arr)
        dlen = self.pkgLen - 6
        rlen = alen if (alen < dlen) else dlen
        for i in range(4, rlen + 4):
            # print(i,arr[i-4])
            self.buf[i] = arr[i - 4]
        self.setCrc()

    def setDataToOff(self):
        for i in range(4, self.pkgLen - 2):
            self.buf[i] = 0
        self.buf[self.pkgLen - 2] = (self.pixStyle + self.frameWidth + self.frameHeight) & 0xff
        # self.setCrc()

    def setDataToOn(self):
        for i in range(4, self.pkgLen - 2):
            self.buf[i] = 255
        self.setCrc()

    def setDataToRGBW(self, r=0, g=0, b=0, w=0):
        sty = self.pixStyle & 0x0f
        if sty == 1:
            for i in range(0, int((self.pkgLen - 6) / sty)):
                self.buf[i * sty + 4] = r
        if sty == 2:
            for i in range(0, int((self.pkgLen - 6) / sty)):
                self.buf[i * sty + 4] = r
                self.buf[i * sty + 5] = g
        if sty == 3:
            for i in range(0, int((self.pkgLen - 6) / sty)):
                self.buf[i * sty + 4] = r
                self.buf[i * sty + 5] = g
                self.buf[i * sty + 6] = b
        if sty == 4:
            for i in range(0, int((self.pkgLen - 6) / sty)):
                self.buf[i * sty + 4] = r
                self.buf[i * sty + 5] = g
                self.buf[i * sty + 6] = b
                self.buf[i * sty + 7] = w
        self.setCrc()

    def packBytes(self):
        packstyle = str(self.pkgLen) + 'B'  # B 0-255     b 0-127

        req = struct.pack(packstyle, *self.buf)
        return req


if __name__ == '__main__':
    d = FrameData(0x14, 10, 2)

    print("init", d.buf)
    d.setDataToOn()
    print("on", d.buf)
    d.setDataToRGBW(255)
    print("r", d.buf)
    d.setDataToRGBW(0, 255)
    print("g", d.buf)
    d.setDataToRGBW(0, 0, 255)
    print("b", d.buf)
    d.setDataToRGBW(0, 0, 0, 255)
    print("w", d.buf)
    d.setDataToRGBW(255, 255, 255, 255)
    print("rgbw", d.buf)
    d.setDataToArray([0, 0, 0])
    print("000", d.buf)
    import time
