This is the source code for the VESC DC/BLDC/FOC controller. Read more at  
http://vesc-project.com/
## bldc-mjvesc_v01_3.40


## eclipse setting @ windows, 2019-02-08
1. install jdk

   https://www.oracle.com/technetwork/java/javase/downloads/index.html

2. install eclipse

   https://www.eclipse.org/downloads/

3. install 'gnu-arm-mcu' plugin at help -> eclipse market place

4. go to 'Packs' perspective, click refresh and install STMicroelectronics->Keil->STM32F4xx_DFP

5. install nodejs, npm

    https://nodejs.org/en/download/

6. install xpm

    `npm install --global xpm`
    
7. install ARM toolchain

    `xpm install --global @gnu-mcu-eclipse/arm-none-eabi-gcc`
    
8. install openocd

    `xpm install --global @gnu-mcu-eclipse/openocd`
    
9. install windows build tools

    `xpm install --global @gnu-mcu-eclipse/windows-build-tools`
    
    
    
## eclipse setting @ ubuntu, 2019-02-08
1. install jdk

    `sudo apt install openjdk-11-jdk`

2. install eclipse

   https://www.eclipse.org/downloads/

3. install 'gnu-arm-mcu' plugin at help -> eclipse market place

4. go to 'Packs' perspective, click refresh and install STMicroelectronics->Keil->STM32F4xx_DFP

5. install nodejs, npm

    `sudo apt-get install nodejs-dev node-gyp libssl1.0-dev`
    
    `sudo apt-get install npm`

6. install xpm

    `sudo npm install --global xpm`
    
7. install ARM toolchain

    `xpm install --global @gnu-mcu-eclipse/arm-none-eabi-gcc`
    
8. install openocd

    `xpm install --global @gnu-mcu-eclipse/openocd`
    
    
