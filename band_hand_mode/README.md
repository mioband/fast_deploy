# fast_deploy/band_hand_mode
Данный репозиторий включает в себя последнюю рабочую версию исходного кода для браслета, а также инструкции по его редактированию и загрузке в память микроконтроллеров ESP32 и GD32, используемых в браслете.

# Редактирование и загрузка исходного кода в память ESP32
## Необходимые программы и драйверы
1. [Microsoft Visual Studio Code](https://code.visualstudio.com/download)
2. [Дополнение PlatformIO для VS Code](https://r13-project.ru/2021/01/09/установка-platformio-в-visual-studio-code/)
3. Драйверы CH340. Если ОС автоматически не установила драйверы данной микросхемы, [то загрузите и установите их самостоятельно](https://voltiq.ru/ch340g-driver-windows-7-10/)
4. Доступ в Интернет для загрузки п.1-3.

## Редактирование и загрузка
1. Скопируйте дирректорию *fast/deploy/band_hand_mode/brasletSmart* в удобную для Вас папку, чтобы избежать изменений рабочего проекта. После добавления проекта в среду VS Code, обратите внимание, что имя проекта появилось на нижней панели среды разработки (см. ниже, выделено красным прямоугольником):

![fig.1](https://github.com/mioband/fast_deploy/blob/main/band_hand_mode/images/fig1.png?raw=true)

2. Для редактирования исходного кода откройте файл, расположенный по пути *brasletSmart/src/main.cpp*, конфигурационный файл расположен по пути *brasletSmart/src/config.hpp*.

**MAC-адреса устройств**
- первая верия ответки (из двух отладочных плат): {0x80, 0x7D, 0x3A, 0x99, 0x85, 0x14}
- ответка в черном корпусе: {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xF4}
- ответка без корпуса (с оригинальным STM32): 0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xF0}
- машинка: {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xEC}

3. Перед загрузкой кода укажите порт браслета в файле *brasletSmart/platformio.ini* в переменной **upload_port**.
4. Для загрузки исходного кода в память ESP32 нажмите на стрелку, расположенную на нижней панели (см. ниже, стрелка выделена красным прямоугольником):

![fig.2](https://github.com/mioband/fast_deploy/blob/main/band_hand_mode/images/fig2.png?raw=true)

**Зажмите кнопку, чтобы загрузка прошла успешно! Отпускайте кнопку после того, как все светодиоды браслета стали излучать свет!**

![fig.3](https://github.com/mioband/fast_deploy/blob/main/band_hand_mode/images/fig3.png?raw=true)

# Редактирование и загрузка исходного кода в память GD32
## Необходимые программы
1. [STM32CubeIDE v1.8.0](https://www.st.com/en/development-tools/stm32cubeide.html)
2. [Flash Loader Demonstrator](https://istarik.ru/file/winstmflash.zip).

## Установка ПО
1. STM32CubeIDE v1.8.0 **без VPN или Tor не загрузить**. Для загрузки необходимо указать адрес электронной почты, на которую затем придет письмо с ссылкой для загрузки программы. Эта среда используется для редактирования и сборки проекта, а также **создания бинарного файла прошивки**.
2. Flash Loader Demonstrator. Распакуйте архив *winstmflash.zip* в удобную для Вас дирректорию.

## Редактирование исходного кода
1. Скопируйте дирректорию *fast_deploy/band_hand_mode/armband_sensors_Bottom_v2* в удобную для Вас папку, чтобы избежать изменений рабочего проекта.
2. При первом запуске STM32CubeIDE среда попросит указать папку, в которой будут находиться проекты. Можно создавать несколько таких папок, для данного проекта достаточно одной.
3. Чтобы открыть в STM32CubeIDE исходный проект, выберите пункт верхней панели меню *File->Open Projects from File System...*. Далее в открывшемся окне указываете проект *armband_sensors_Bottom_v2*, нажав на кнопку *Directory*:
![fig.4](https://github.com/mioband/fast_deploy/blob/main/band_hand_mode/images/fig4.png?raw=true)

Для завершения процесса добавления проекта нажмите на кнопку *Finish* в правой нижней части окна:
![fig.5](https://github.com/mioband/fast_deploy/blob/main/band_hand_mode/images/fig5.png?raw=true)

4. Для редактирования исходного кода откройте файл, расположенный по пути *armband_sensors_Bottom_v2/Core/src/main.c*.
5. Для сборки проекта и генерирования бинарного файла нажмите ПКМ на проекте *armband_sensors_Bottom_v2* в окне дерева проектов *Project Explorer*, затем выберите пункт *Build Project*.
6. Прежде чем приступать к загрузке кода в память GD32, следует загрузить специальный код в ESP32, чтобы было возможно передавать данные к GD32 через ESP32. Для этого скопируйте в удобную для Вас дирректорию проект *fast_deploy/band_hand_mode/esp32_uart_bridge_to_stm32*, после чего добавьте и откройте его в среде **VS Code**.
7. Далее загружаете данный проект в соответствии с пунктами инструкции для загрузки кода в ESP32. Не забудьте, что необходимо зажать кнопку для успешной загрузки. **Обратите внимание**, что с данным проектом светодиоды гореть не будут. Чтобы понять, что загрузка прошла успешно, посмотрите в консоль VS Code. После успешной загрузки там должны быть подобные строчки:

![fig.6](https://github.com/mioband/fast_deploy/blob/main/band_hand_mode/images/fig6.png?raw=true)

8. После загрузки кода в ESP32 открываете программу Flash Loader Demonstrator. Выбираете порт устройства в пункте *Port Name*, **устанавливаете скорость 9600** в пункте *Baud Rate*, остальное - по умолчанию, затем нажимаете на кнопку *Next*.
9. В следующих двух окнах приложения (первое - с изображением светофора, второе содержит таблицу с выбором типа ядра микроконтроллера) нажимаете *Next.*
10. В следующем окне выбирите пункт *Download from file*, затем с помощью кнопки *...* (см. ниже, выделено красным) укажите путь к бинарному файлу прошивки, путь к которому следующий: *armband_sensors_Bottom_v2/Debug/armband_sensors_Bottom_v2.bin*.

![fig.7](https://github.com/mioband/fast_deploy/blob/main/band_hand_mode/images/fig7.png?raw=true)

11. Для загрузки бинарного файла нажмите на кнопку *Next* в нижней части окна программы (см. выше, выделено темно-зеленым). После успешной загрузки можете закрывать программу.
12. Последним шагом **загрузите последнюю версию прошивки ESP32 браслета вместо проекта** *esp32_uart_bridge_to_stm32*!
