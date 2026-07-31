
## 1. Install the NuttX
  
https://nuttx.apache.org/docs/latest/quickstart/index.html

Create a folder `nuttxspace/` and clone inside

```bash
git clone https://github.com/apache/nuttx.git nuttx

git clone https://github.com/apache/nuttx-apps apps
```

Go to `nuttx.md`

Uzinp seems neeeded sometimes bu Nuttx so install it

```bash
sudo apt install unzip
```
## Build NuttX and falsh it

### Setup your board

In the `nuttxspace/nuttx/` folder:

Search for your board

`./tools/configure.sh -L | grep h7`

Set the app folder and configure .config

```bash
# For nucleo-h753zi or h743zi

./tools/configure.sh -l nucleo-h743zi:nsh

# For the shirley-fc-dev-board

./tools/configure.sh -l ../boards/arm/stm32h7/shirley-fc-dev-board/configs/nsh
```

### Config the board

Show the menu config (/nuttxspace/nuttx)

`sudo make menuconfig`

This should open an interactive menu to configure the NuttX build

To clean a config:

`make distclean`
or
`make clean`

then create a new .config

For the nucleo boards you can run:
`./tools/configure.sh -l nucleo-h743zi:nsh`

and you can go again


To check the binary size

```bash

arm-none-eabi-size nuttx

# => FLASH text+data(+rodata)

# => RAM bss+data

# - FLASH 128Kbytes (128*2^10: 131072bytes - binary shit/BS) in nucleo-STM32G431RB

# - SRAM 32KBytes (128*2^10: 32768bytes) in nucleo-STM32G431RB

```

To have a pretty render

```bash

arm-none-eabi-size nuttx | tail -1 | awk -v flash=131072 -v sram=32768 '{

flash_used = $1 + $2;

sram_used = $2 + $(3);

printf("Mem region Used Size Region Size %%age Used\n");

printf(" flash: %10d B %5d KB %6.2f%%\n", flash_used, flash/1024, 100*flash_used/flash);

printf(" sram: %10d B %5d KB %6.2f%%\n", sram_used, sram/1024, 100*sram_used/sram);

}'

```

  

Size experience sharing for a stm32g431rb:

At first build of the OS the size is

```bash

Mem region Used Size Region Size %age Used

flash: 128420 B 128 KB 97.98%

sram: 9504 B 32 KB 29.00%

```

We can deactivate the following:

- `Application Configuration -> Testing -> OS test example`, so deactivate it (unsing `n` or `y`) not `space` (-44.13% flash/ -5.27% RAM)

- `Application Configuration -> System Libraries and NSH Add-Ons -> system 'dd' command` (-3.61% flash/ -0.06% RAM)

- `Application Configuration -> System Libraries and NSH Add-Ons -> readline() Support`

- `Binary Loader -> Disable BINFMT support` (-0.08% flash/ -0.05% RAM)

- You can go further, and its necessary for a finished app

It drops to

```bash

Mem region Used Size Region Size %age Used

flash: 70096 B 128 KB 53.48%

sram: 9040 B 32 KB 27.59%

```

which can be used reliably. So test the OS perf and then deactivate it.

  

Usefull params experience for a stm32g431rb:

- `Application Configuration -> System Libraries and NSH Add-Ons -> readline() Support`: can be used to activate nsh `tab`, `backspace`, ...

- `Build Setup -> Debug Options -> Define NDEBUG globally` to deactivate

- `Build Setup -> Debug Options -> Stack coloration`

- `Build Setup -> Debug Options -> Generate stack usage information`

- `RTOS Features -> Stack BackTrace` This one and the 3 previous seems to take ~8-10%

Using these future, we are at

```bash

Mem region Used Size Region Size %age Used

flash: 84620 B 128 KB 64.56%

sram: 9228 B 32 KB 28.16%

```


### Flash the board


Clean and build the RTOS with the given .config

`sudo make clean`

(-j is to allow parallel compiling)

`sudo make -j`

(to avoid logs)

`sudo make -j > /dev/null`

To connect to a board
```bash
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c 'adapter speed 100' -c 'init' -c 'targets' -c 'shutdown' 
```
=> In long term, a openOCD config file will be needed to automatically reduce speed

For a nucleo board:
```bash
openocd -f board/st_nucleo_h743zi.cfg \
  -c "init; reset halt" \
  -c "program nuttx.bin 0x08000000 verify reset exit"
```

### To use the nuttshell to debug

Source

https://nuttx.apache.org/docs/latest/applications/nsh/nsh.html

Once Flashed, connect using

```bash
picocom -b 115200 /dev/ttyACM0
```
or on MacOS:
run first `ls /dev/cu.*`to find the device then 
```bash
picocom -b 115200 /dev/cu.usbmodem1303
```

Tips on picocom

```bash
ctrl+a -> ctrl+h
```

## Enable an app

For example we want to enable the `hello`example app:
The app is in:
`/apps/example/hello/`

acces the menuconfig and enable it:
```
Application Configuration  
└── Examples  
└── [*] Hello World example
```
 then in nuttshell you can call the app with `hello`
## To change the 'app' folder for 'custom_apps'

To tweak a kconfig parameter

`sudo kconfig-tweak --set-str CONFIG_APPS_DIR ../custom_apps

When needed to change app directory

`./tools/configure.sh -a ../custom_apps nucleo-g431rb:nsh