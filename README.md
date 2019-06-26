# Synopsys ARC盃競賽 
室內舒適度小盒子 -- A醬

## Introduction
* 夏季將近，為了消暑，不論在家裡、工作場所，我們總是離不開冷氣房。這樣不僅浪費電力，也容易造成感冒。在這**環保節能**意識抬頭的情景下，我們除了購買較節能的家電外，是否還有其他方式能更加節省能源呢？
* 丹麥學者P.O. Fanger教授於1972年所發表**人體熱平衡模型**，該模型用來表示人體對於環境中冷、熱的感受。該模型提出，除了溫度，濕度、風速等因素也影響人體對環境舒適度感受。
* ![](https://i.imgur.com/nmDoIey.png)
  M：人體新陳代謝量(W/m2)
  W：外部影響人體之能量(W/m2)
  Icl：衣著隔絕度(m2K/W)
  fcl：衣著面積因素
  ta：氣溫(oC)
  tr：平均輻射溫度(oC)
  var：相對風速(m/s)
  Pa：水氣分壓(Pa)
  hc：對流熱傳係數(W/(m2K))
  tcl：衣著表面溫度(oC)


## Hardware & Software
### Hardware connection
![](https://i.imgur.com/7jLGut2.jpg)

| Embarc | Esp8266 | OLED 128x64 | WindSpeed | DHT11 |
| -------- | -------- | -------- | -------- | -------- |
| PMode (UART1) | TX / RX |  |  |  |
| I2C0 - SCL |  | SCL |  |  |
| I2C0 - SDA |  | SCA |  |  |
| Arduino Extension A0 |  |  | TMP |  |
| Arduino Extension A1 |  |  | RV |  |
|  | D2 |  |  | Data |

### Software structure
![](https://i.imgur.com/hMVhju3.png)


## User manual
### Before Running this Application
The hardware resources are allocated as following table.


| Hardware Resource | Function |
| -------- | -------- |
| Esp8266 NodeMCU | Connect internet and gather the info of DHT11 |
| DHT11 | Gather the inroom information of temperature and humidity |
| Wind Speed Module | Measure the inroom wind speed |

* Download [DHT library](https://github.com/adafruit/DHT-sensor-library) for Arduino

### Run this Application
#### Makefile
* Download Synopsys Arc open-source platform(OSP)
  ```
    $git clone https://github.com/foss-for-synopsys-dwc-arc-processors/embarc_osp.git```
* Move to EMBARC_OSP/example/baremetal
  ```$cd EMBARC_OSP/example/baremetal```
* Download ssd_littleBox_Achan
  ```$git clone https://github.com/Julie0510735/synopsys_ARC.git```
* Move to synopsys_ARC/ssd_littleBox_Achan
  ```$cd synopsys_ARC/ssd_littleBox_Achan```
* Makefile
  ```$makefile```
## Demo video
{%youtube FNsj988beX8 %}
