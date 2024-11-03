"""
送受信器 main.py
"""
from microbit import *
import time

# UARTの初期化（適切な設定に変更してください）
uart.init(baudrate=115200)

cmd = ""
while True:
    if uart.any():  # データが受信されているか確認
        if button_a.is_pressed():
            uart.write("S\n")
        if button_b.is_pressed():
            uart.write("C\n")

        char = uart.read(1)  # 1文字を読み取り
        char = str(char, 'UTF-8')  # バイト列を文字列に変換

        if char == '\n':  # 改行の場合
            x_strength = accelerometer.get_x()
            res = cmd + str(x_strength)
            uart.write(res + "\n")  # 応答を送信
            cmd = ""  # コマンドをリセット
        else:
            # 受信した文字がコマンドの一部であれば追加
            cmd += char.strip()  # 余計な空白を削除して追加

    time.sleep(0.01)  # 短くして受信処理を迅速に
