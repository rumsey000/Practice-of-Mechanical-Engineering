# 機械工程實務第二十六組專題
## 專題目標
本學年度機械⼯程實務將以分組⽅式完成⼀⾞輛的製作，此⾞輛利⽤環境⾵⼒作為驅動源，製作可將⾵⼒轉換為動⼒(如⾵帆)之機構，藉由控制該機構使⾞輛⾏進並轉向，完成⾃主⾏進任務，故名為AeroRide。
## 程式開發
### IMU(mpu6050)
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
### encoder
