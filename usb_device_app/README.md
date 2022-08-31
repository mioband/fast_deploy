# fast_deploy/usb_device_app
Данный репозиторий содержит последнюю рабочую версию исходного кода для устройства, а также инструкции по его редактированию и загрузке в память микроконтроллеров ESP32 и GD32.

# Редактирование и загрузка исходного кода в память GD32
## Необходимые программы и драйверы
1. STM32CubeIDE v1.8.0
2. [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html#get-software)
3. Драйвер ST-Link/V2/V2-1. Если не установится автоматически при подключении программатора, то [драйвер можно найти тут](https://www.st.com/en/development-tools/stsw-link009.html#get-software), установите самостоятельно

## Дополнительное аппаратное обеспечение
1. [ST-Link V2](https://roboshop.spb.ru/programmator/st-link-v2)


## Установка ПО
Для загрузки STM32CubeIDE v1.8.0, STM32CubeProgrammer и драйверов ST-Link может потребоваться использование браузера Tor или VPN.

## Редактирование исходного кода
Аналогично инструкции, представленной в *fast_deploy/band_hand_mode/README.md*.

## Загрузка бинарного файла в память GD32
1. Сперва подключите выводы ST-Link к отверстиям применого устройства в соответствии со следующими обозначениями:

*3* <-> *3.3V*

*G* <-> *GND*

*C* <-> *SWCLK*

*D* <-> *SWDIO*

2. Копируете проект *esp_stm_usb_GD32_transceiver* в удобное место и открываете его в STM32CubeIDE, собираете проект.
3. Затем запускаете программу STM32CubeProgrammer, нажимаете на кнопку *Open file* и выбираете бинарный файл прошивки GD32 *esp_stm_usb_GD32_transceiver/Debug/esp_stm_usb_GD32_transceiver.bin*:

![fig.1](https://github.com/mioband/fast_deploy/blob/main/usb_device_app/images/fig1.png?raw=true)

4. Подключаетесь к микроконтроллеру через кнопку *Connect*:

![fig.2](https://github.com/mioband/fast_deploy/blob/main/usb_device_app/images/fig2.png?raw=true)

5. Наконец, для загрузки нажимаете на кнопку *Download*:

![fig.3](https://github.com/mioband/fast_deploy/blob/main/usb_device_app/images/fig3.png?raw=true)


6. (не обязательно) отключаетесь от GD32 с помощью кнопки *Disconnect*

# Редактирование и загрузка исходного кода в память ESP32
## Необходимые программы и драйверы
1. [Microsoft Visual Studio Code](https://code.visualstudio.com/download)
2. [Дополнение PlatformIO для VS Code](https://r13-project.ru/2021/01/09/установка-platformio-в-visual-studio-code/)
3. Драйвер [Virtual COM Port](https://www.st.com/en/development-tools/stsw-stm32102.html#get-software).
4. PyCharm Community.
5. [Библиотека esptool для Python](https://pypi.org/project/esptool/)
6. [Библиотека stm32loader для Python](https://pypi.org/project/stm32loader/)

## Редактирование и загрузка
Инструкция по редактированию исходных файлов в VS Code аналогична той, что представлена в файле *fast_deploy/band_hand_mode/README.md*. Имя проекта: *brasletSmartServ*.

### Загрузка
1. Копируете проект *mio_band_update* в дирректорию по Вашему усмотрению, чтобы избежать внесения нежелательных изменений.
2. Подключаете устройство в USB-разъем, определяете порт.
3. Откройте в PyCharm проект *mio_band_update*, в файле *main_band_uploader.py* и на строке **68** в качестве первого аргумента метода *esp_upload* впишите имя и номер порта, к которому подключено обновляемое устройство. Например, 
```
band_up.esp_upload('COM7', './firmwares/firmware.bin')
```

или 

```
band_up.esp_upload('/dev/ttyUSB0', './firmwares/firmware.bin')
```

4. После сборки platfromio-проекта для ESP32 перемещаете следующие файлы в соответствующие дирректории:

*/brasletSmartServ/.pio/build/esp32doit-devkit-v1/firmware.bin* -> */mio_band_update/firmwares/*

*/brasletSmartServ/.pio/build/esp32doit-devkit-v1/partitions.bin* -> */mio_band_update/config/*

5. Запускаете выполнение программы из файла *main_band_uploader.py*.
6. Дожидаетесь окончания загрузки, в процессе загрузки на устройстве будет гореть один светодиод. Также можете наблюдать прогресс загрузки в консоли PyCharm.
7. После завершения работы программы требуется подождать около 10 секунд, чтобы устройство автоматически перезагрузилось. По чередующемуся миганию двух светодиодов можно понять, что оно готово к работе.
