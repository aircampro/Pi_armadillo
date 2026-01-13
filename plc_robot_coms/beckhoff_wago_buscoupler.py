#!/usr/bin/python
#
# Interface to beckhoff and wago bus couplers https://github.com/Nonannet/pyhoff
#
import inspect
import pyhoff as pyhoff
import pyhoff.devices as devices
from pyhoff.devices import KL2404, KL2424, KL9100, KL1104, \
    KL3202, KL4002, KL9188, KL3054, KL3214, KL4004, KL9010, BK9050, WAGO_750_352 \
	DigitalInputTerminal, DigitalOutputTerminal, AnalogInputTerminal, AnalogOutputTerminal

def test_readme_example():
    # connect to the BK9050 by tcp/ip on default port 502
    bk = BK9050("172.16.17.1")

    # add all bus terminals connected to the bus coupler
    # in the order of the physical arrangement
    bk.add_bus_terminals(KL2404, KL2424, KL9100, KL1104, KL3202,
                         KL3202, KL4002, KL9188, KL3054, KL3214,
                         KL4004, KL9010)

    # Set 1. output of the first KL2404-type bus terminal to hi
    bk.select(KL2404, 0).write_coil(1, True)

    # read temperature from the 2. channel of the 2. KL3202-type
    # bus terminal
    t = bk.select(KL3202, 1).read_temperature(2)
    print(f"t = {t:.1f} °C")

    # Set 1. output of the 1. KL4002-type bus terminal to 4.2 V
    bk.select(KL4002, 0).set_voltage(1, 4.2)

    # write a pulse to outputs on beckhoff
    for bt in bk.bus_terminals:
        if isinstance(bt, DigitalOutputTerminal):
            for channel in range(1, bt.parameters.get('output_bit_width', 0) + 1):
                bt.write_coil(channel, True)
				time.sleep(2)
                bt.write_coil(channel, False)	

    # connect to WAGO bus coupler
    terminal_classes: list[type[pyhoff.BusTerminal]] = []
    for n, o in inspect.getmembers(devices):
        if inspect.isclass(o) and o not in [DigitalInputTerminal,
                                            DigitalOutputTerminal,
                                            AnalogInputTerminal,
                                            AnalogOutputTerminal]:
            if issubclass(o, pyhoff.BusTerminal):
                print(n)
                terminal_classes.append(o)
	wg = devices.WAGO_750_352("172.16.17.2", 11255, terminal_classes, timeout=0.001)

    # write a 2s pulse to outputs on wago
    for bt in wg.bus_terminals:
        if isinstance(bt, DigitalOutputTerminal):
            for channel in range(1, bt.parameters.get('output_bit_width', 0) + 1):
                bt.write_coil(channel, True)
				time.sleep(2)
                bt.write_coil(channel, False)
