# ESP32-TFT-LCD-Screen-Parallel
480 * 320 TFT LCD Screen with ESP32 using parallel communication

![Alt Text](https://raw.github.com/kpatel122/ESP32-TFT-LCD-Screen-Parallel/main/docs/pics/esp32.png)
![Alt Text](https://raw.github.com/kpatel122/ESP32-TFT-LCD-Screen-Parallel/main/docs/pics/screen0.jpg)
![Alt Text](https://raw.github.com/kpatel122/ESP32-TFT-LCD-Screen-Parallel/main/docs/pics/screen1.jpg)

http://www.lcdwiki.com/3.95inch_Arduino_Display-Mega2560

Adapted the original arduino mega library to ESP32. This is a work in progress. Utimately it would be great to get the TFT_eSPI library by bodmer (which in theory does support ESP32 in parallel mode- but I couldn't get it to work with this screen). For now I have merged the working setup code from the lcdwiki with the faster writes of the TFT_eSPI lib
Ensure the pins used for the data bus are <32 since the modifications used a single register (GPIO.out_w1tc) for writes- refer to the ESP32 tech spec for more info regarding this
