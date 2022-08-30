# fast_deploy/band_wrist_mode
Все исходные файлы и инструкции аналогичны тем, которые находятся в дирректории *fast_deploy/band_hand_mode*.

**Важное отличие между репозиториями**: в файле *fast_deploy/band_hand_mode/brasletSmart/src/main.cpp* на 39 строке присвойте 1 переменной *gesture_mode*:
```
volatile uint8_t gesture_mode = 1;
```

Это необходимо для определения жеста по сигналу с одного электрода. 
