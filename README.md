compile on hb-server

# get juicer code
cd ~/bin
git clone https://github.com/ryanmillerwork/juicer.git
cd juicer

# if using esp32-s3 reverse tft:
sudo nano ~/.arduino15/packages/esp32/hardware/esp32/3.1.1/variants/adafruit_feather_esp32s3_reversetft/pins_arduino.h
change to "#define USB_PRODUCT "juicer3"

# compile
~/bin/arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32s3_reversetft --build-path ~/bin/juicer/juice_pump3/build ~/bin/juicer/juice_pump3/

# push back
git add juice_pump3/build/
git push origin main




# on homebase
cd ~/code

sudo usermod -a -G dialout lab
sudo curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
~/code/bin/arduino-cli core install esp32:esp32

git clone https://github.com/ryanmillerwork/juicer.git
cd ~/code/juicer/juice_pump3
~/code/bin/arduino-cli board list 
~/code/bin/arduino-cli upload --fqbn esp32:esp32:adafruit_feather_esp32s3_reversetft -p /dev/ttyACM0 --input-dir ./build/ .
