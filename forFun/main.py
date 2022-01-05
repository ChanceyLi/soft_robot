import random
import win32gui as a
import win32con as b
import win32clipboard as c
import time

msg = ['扩列+', '+++', '扩充列表！', 'cpdd', '那好吧', '加加加']
face = ['/pz', '/wx', '/xyx', '/huaix', '/doge']
n = 100
name = '优质扩列cp交友群'
msg_len = len(msg) - 1

for i in range(n + 1):

    num = random.randint(0, msg_len)
    f_num = random.randint(0, len(face) - 1)
    isFace = random.randint(0, 1)
    m = ''
    if isFace:
        m = msg[num] + face[f_num]
    else:
        m = msg[num]
    c.OpenClipboard()
    c.EmptyClipboard()
    c.SetClipboardData(b.CF_UNICODETEXT, m)
    c.CloseClipboard()
    handle = a.FindWindow(None, name)

    if handle != 0:
        a.SendMessage(handle, 770, 0, 0)
        a.SendMessage(handle, b.WM_KEYDOWN, b.VK_RETURN, 0)

    print('发送成功！', i)
    time.sleep(12)
