# using github actions:

# automatically compiles on pushes. to make sure the release is stored, tag it:

git tag v3.0.0
git push origin v3.0.0


# to pull the last tagged version
gh release list
gh release download v3.0.0 -p "*.bin"

OR

wget https://github.com/ryanmillerwork/juicer/releases/download/v3.0.0/juice_pump3.ino.bin

# to pull the latest 
gh run list
gh run download RUN_ID --name compiled-binary


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

git commit -m "Add compiled ESP32 firmware build"

git push origin main



# on homebase
# initial setup

cd ~/code

sudo usermod -a -G dialout lab

sudo curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

~/code/bin/arduino-cli core install esp32:esp32

git clone https://github.com/ryanmillerwork/juicer.git

# just to upload code

unplug/replug usb to microcontroller

cd ~/code/juicer/juice_pump3

~/code/bin/arduino-cli board list 

~/code/bin/arduino-cli upload --fqbn esp32:esp32:adafruit_feather_esp32s3_reversetft -p /dev/ttyACM0 --input-dir ./build/ .

sudo systemctl restart dserv

# calibrating

put exactly 500 ml in container

screen /dev/ttyACM0 115200

{"do": {"calibration": {"n": 600, "on": 500, "off": 500}}}

(wait 10 minutes, measure amount of fluid dispensed [f])

if f=200, ml/s = 200/(600*0.5) = 0.67 ml/s

{"set": {"flow_rate": 0.67}}







