# examples of use of the pymeasure library
# ref :- https://pymeasure.readthedocs.io/en/latest/introduction.html
# https://pymeasure.readthedocs.io/en/latest/api/instruments/
#

# Yokogawa power supply control
#
from pymeasure.instruments.yokogawa import Yokogawa7651
yoko = Yokogawa7651("GPIB::1")
yoko.apply_current()                # Sets up to source current
yoko.source_current_range = 10e-3   # Sets the current range to 10 mA
yoko.compliance_voltage = 10        # Sets the compliance voltage to 10 V
yoko.source_current = 0             # Sets the source current to 0 mA
yoko.enable_source()                # Enables the current output
yoko.ramp_to_current(5e-3)          # Ramps the current to 5 mA
yoko.shutdown()                     # Ramps the current to 0 mA and disables output

# VellemanK8090 Relay control
#
from pymeasure.instruments.velleman import VellemanK8090, VellemanK8090Switches as Switches
import time

instrument = VellemanK8090("ASRL1::INSTR")
# Get status update from device
last_on, curr_on, time_on = instrument.status
# Toggle a selection of channels on
instrument.switch_on = Switches.CH3 | Switches.CH4 | Switches.CH5
time.sleep(2)
instrument.switch_on = Switches.CH1 
time.sleep(2)
instrument.switch_off = Switches.CH3 | Switches.CH4 
time.sleep(2)
instrument.switch_off = Switches.CH1 | Switches.CH5

# Toptica IBeam Smart Laser diode control
#
from pymeasure.instruments.toptica import IBeamSmart

laser = IBeamSmart("SomeResourceString")
laser.emission = True
laser.ch_2.power = 1000  # µW
laser.ch_2.enabled = True
laser.shutdown()