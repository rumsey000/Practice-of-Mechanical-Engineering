# 機械工程實務第二十六組專題
## 專題目標
本學年度機械⼯程實務將以分組⽅式完成⼀⾞輛的製作，此⾞輛利⽤環境⾵⼒作為驅動源，製作可將⾵⼒轉換為動⼒(如⾵帆)之機構，藉由控制該機構使⾞輛⾏進並轉向，完成⾃主⾏進任務，故名為AeroRide。
## 程式開發
### IMU
#### BY USING DMP
- 量測三軸加速度及三軸角速度，並使用MPU6050的DMP硬體來將前面提及的六項整合演算出yaw、roll和pitch的角度
- 主要使用 [ic2devlib](https://github.com/jrowberg/i2cdevlib) 這個開源程式庫內的檔案進行自動校正和資料讀取
#### BY USING KALMAN FILTER
- 量測三軸加速度及三軸角速度
- 重力加速度在三軸加速度的分量可算出yaw和roll的傾角，但剛開始測量時可能誤差較大。
- 角速度對時間的積分項亦能得出yaw和roll的傾角，但測量時間愈久誤差愈大。
- 使用卡爾曼濾波器來將前兩項融合出新的yaw和roll的傾角，其優勢在於可將其的缺點互相彌補，因而較為精準。
- 主要使用 [TKJ Electronics](https://github.com/TKJElectronics/KalmanFilter) 這個開源程式庫內的檔案進行資料讀取
- 校正部分則另外參考 [這個網站](https://wired.chillibasket.com/2015/01/calibrating-mpu6050/) 的程式並將讀取資料的函數改寫為上述開源程式庫的I2C通訊，將這個校正程式先跑一次後，再填入上述程式的偏移量中。
### MOTOR
#### DC motor
-期中使用的馬達，透過程式調整PID參數來達成角度控制
#### Servo motor
-期末使用的馬達，直接對MG996R輸PWM訊號來讓控制角度

## 參考網站
### ARDUINO
- [Arduino uno R3](https://docs.arduino.cc/resources/datasheets/A000066-datasheet.pdf)
- [SerialPlot（序列埠繪圖家）工具軟體（二）：安裝與執行SerialPlot - 超圖解系列圖書](https://swf.com.tw/?p=1591)
- [SerialPlot - Realtime Plotting Software | Hackaday.io](https://hackaday.io/project/5334-serialplot-realtime-plotting-software)
- [[C觀念] volatile 的用法和用意 @ 不會的就放這邊 :: 痞客邦 ::](https://anal02.pixnet.net/blog/post/117485340-%5Bc%5D-volatile-%E7%9A%84%E7%94%A8%E6%B3%95%E5%92%8C%E7%94%A8%E6%84%8F)
- [浅析C语言之uint8_t / uint16_t / uint32_t /uint64_t-CSDN博客](https://blog.csdn.net/mary19920410/article/details/71518130)
- [Arduino教程：使用millis()代替delay() – 趣讨教](https://www.qutaojiao.com/21429.html)
- [Arduino自定義庫的編寫 - 台部落](https://www.twblogs.net/a/5b833c9f2b71771e35c1ccf3)
### IMU
- [MPU-6000-Register-Map1.pdf](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [I2C总线和通信协议详解](https://zhuanlan.zhihu.com/p/678229227)
- [Calibrating & Optimising the MPU6050 – chillibasket](https://wired.chillibasket.com/2015/01/calibrating-mpu6050/)
- [关于MPU6050的数据获取、分析与处理_matlab处理mpu6050原始数据-CSDN博客](https://blog.csdn.net/acktomas/article/details/89087174)
- [ASCII Table / character codes – SS64.com](https://ss64.com/ascii.html)
- [Pitch and Roll Estimation](https://www.nxp.com/docs/en/application-note/AN3461.pdf)
- [MPU6050 DMP freezes after random duration · Issue #519 · jrowberg/i2cdevlib · GitHub](https://github.com/jrowberg/i2cdevlib/issues/519)
- [position - 使用加速度计和陀螺仪计算位移 (MPU6050) - IT工具网](https://www.coder.work/article/6199777)
### DC MOTOR
- [NFP-JGA25-370-1260  12V DC datasheetModel ](https://nfpmotor.com/custom-gear-motor-6v12v24v-model-nfp-jga25-370)
- [3V, 6V, 12V DC Micro Metal Gearmotor JGA12-N20 | NFPmotor.com](https://nfpmotor.com/3v-6v-12v-dc-micro-metal-gearmotor-model-nfp-jga12-n20)
- [[RWG] 全新 高扭力 GM25-370 25GA370 直流減速馬達 帶編碼器 測速碼盤 | 蝦皮購物](https://shopee.tw/-RWG-%E5%85%A8%E6%96%B0-%E9%AB%98%E6%89%AD%E5%8A%9B-GM25-370-25GA370-%E7%9B%B4%E6%B5%81%E6%B8%9B%E9%80%9F%E9%A6%AC%E9%81%94-%E5%B8%B6%E7%B7%A8%E7%A2%BC%E5%99%A8-%E6%B8%AC%E9%80%9F%E7%A2%BC%E7%9B%A4-i.14363185.4645809394?xptdk=3c304600-352a-41ea-b997-f45115aef90b)
- [Stepper Motor with L298N and Arduino Tutorial (4 Examples)](https://www.makerguides.com/l298n-stepper-motor-arduino-tutorial/)
- [Arduino | 旋轉編碼器模組使用](https://hugheschung.blogspot.com/2018/06/arduino-1.html)
### SERVO MOTOR
- [MG90S_Tower-Pro.pdf](https://www.electronicoscaldas.com/datasheet/MG90S_Tower-Pro.pdf)
- [sg90_datasheet.pdf](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf)
- [[Arduino範例] SG90 Servo伺服馬達](https://blog.jmaker.com.tw/arduino-servo-sg90/)
- [伺服馬達，回授控制入門 - 互動的風格 - Medium](https://medium.com/electronics-%E4%BA%92%E5%8B%95%E9%9B%BB%E5%AD%90/%E5%AF%A6%E9%AB%94%E9%81%8B%E7%AE%97week4-4943b619f476)
- [How to interface SG90 servo motor with Arduino](https://www.simplyiotsensors.com/2021/10/how-to-use-sg-90-servo-motor-with-Arduino.html#:~:text=SG90%20servo%20motor%20runs%20between%204.8%20%E2%80%93%206,of%20the%20motor%20it%20will%20consume%20100-250%20mA.)
### STEP MOTOR
- [[Arduino範例] ULN2003驅動板+28BYJ-48步進馬達](https://blog.jmaker.com.tw/uln2003-28byj-48/)
- [【產品/介紹】ULN2003 驅動板 + 28BYJ-48 步進馬達 - 台灣樹莓派](https://piepie.com.tw/8199/uln2003-stepper-motor-drive#:~:text=%E5%B7%A5%E4%BD%9C%E9%9B%BB%E5%A3%93%EF%BC%9A5VDC%20%E7%9B%B8%E4%BD%8D%EF%BC%9A4%20%E9%80%B1%E6%9C%9F%EF%BC%9A100Hz%20%E9%80%9F%E5%BA%A6%E8%AE%8A%E5%8C%96%E7%8E%87%EF%BC%9A1%2F64,%E6%91%A9%E6%93%A6%E5%8A%9B%E7%9F%A9%EF%BC%9A600-1200%20gf.cm%20%E5%BC%95%E5%85%A5%E8%BD%89%E7%9F%A9%EF%BC%9A300%20gf.cm)
### SWITCH
- [《Arduino入門》第五篇：按鍵開關的使用](https://blog.jmaker.com.tw/arduino-buttons/)
- [Debounce on a Pushbutton | Arduino Documentation](https://docs.arduino.cc/built-in-examples/digital/Debounce/)
- [上拉電阻 - 維基百科，自由的百科全書](https://zh.wikipedia.org/zh-tw/%E4%B8%8A%E6%8B%89%E7%94%B5%E9%98%BB)
### MERCHANT
- [宇倉電子-全館(已含稅)開發票, 線上商店 | 蝦皮購物](https://shopee.tw/allen_6833?categoryId=100644&entryPoint=cart&itemId=64927673)
- [今華電子網路商城暨實體門市](https://jin-hua.com.tw/index.aspx)
- [廣華電子商城 - 敦華電子材料有限公司_網路郵購-電子材料-電子零件-臺灣 台中市](https://shop.cpu.com.tw/)
- [祥昌電子｜官方購物網 電子零售批發 電子耗材 IC半導體 儀錶 零組件 影音線材 手工具 電源供應器 3C 電子線](https://www.sconline.com.tw/tw/product/index.php?page=1&kind1=&kw=dc%E6%8E%A5%E9%A0%AD)
- [AWG WIRE TABLE](https://ken-gilbert.com/techstuff/AWG_WIRE_TABLE.html)
### SIMULATOR
- [New Arduino Uno Project - Wokwi Simulator](https://wokwi.com/projects/new/arduino-uno)
- [Circuit design Daring Esboo - Tinkercad](https://www.tinkercad.com/things/0wW5GkK5LwL-daring-esboo/editel?tenant=circuits)
- [BLOCK SVG EDIT V20231029](https://www.block.tw/bse/v20231029/)
- [Block Circuit EDIT](https://www.block.tw/bce/)
- [circuito.io](https://www.circuito.io/app?components=512,11021,11028,7654321)
### 3DCAD
- [麵包板](https://grabcad.com/library/mini-breadboard--1)
- [馬達驅動模組](https://grabcad.com/library/l298n-21)
- [18650電池盒](https://grabcad.com/library/4-x-18650-battery-box-1)
- [9V方型電池盒](https://grabcad.com/library/battery-box-9v-boitier-pour-pile-9-v-avec-interrupteur-on-off-1)
- [Arduino板](https://grabcad.com/library/arduino-uno-r3-11)
- [M4螺帽](https://www.mcmaster.com/products/nuts/hex-nuts~/thread-size~m4/)
- [M3螺帽](https://www.mcmaster.com/products/nuts/hex-nuts~/thread-size~m3/)
- [M3螺栓](https://www.mcmaster.com/products/screws/rounded-head-screws~/thread-size~m3/jis-steel-pan-head-screws/)
- [M4螺栓](https://www.mcmaster.com/products/screws/rounded-head-screws~/jis-steel-pan-head-screws/thread-size~m4/thread-size~m4/)
- [銅柱](https://www.mcmaster.com/products/male-female-threaded-hex-standoffs/male-female-threaded-hex-standoffs-6/thread-size~m3/)
### 成果展示
- <http://www.me.ntu.edu.tw/capstone/project.html>
