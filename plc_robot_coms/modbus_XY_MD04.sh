# https://yomon.hatenablog.com/entry/2023/1/xy-md02-pymodbus
# https://github.com/pymodbus-dev/pymodbus
# reads Temperature and Humidity Sensor XY-MD02
# USB-RS485 https://www.amazon.co.jp/dp/B078X5H8H7?tag=kozinsyohyou-22&linkCode=ogi&th=1&psc=1
# $ udevadm info -q property -n /dev/ttyUSB*
# 
# pip install pymodbus click pygments prompt_toolkit
# sudo yum install bc
#
# reg1 is input register 1 count 1 = Temperature
v = client read_input_registers address=2 count=1 unit=1
echo "temperature"
echo "scale=5;$v/10" | bc -l
echo ""
# reg2 is input register 2 count 1 = Humidity
v = client read_input_registers address=2 count=1 unit=1
echo "humidity"
echo "scale=5;$v/10" | bc -l
echo "" 