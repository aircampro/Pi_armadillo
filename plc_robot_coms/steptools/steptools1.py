#!/usr/bin/env python
# =======================================
#
# simple export and print from steptools
#
# =======================================
#
from steptools import step
import sys

# check correct usage
if len(sys.argv)<2:
    print("usage sys.argv[0] <input stpnc file name> <machine style == fanuc haas okuma heidenhain siemens>")
	sys.exit(-1)
	
# Read a STEP-NC file as the project file
fn=str(sys.argv[1]) 
if len(fn.split("."))==1:                # we didnt add the extension for the file
    fn = fn+".stpnc"
DESIGN = step.open_project(fn);
GEN = step.Generate()

# choose style from argumets passed
ms=str(sys.argv[2])
# list of possible styles
# [ 'fanuc', 'haas', 'okuma', 'heidenhain', 'siemens']
#
if [ 'fanuc' == ms ]:
    GEN.set_style('fanuc')                # use fanuc style
elif [ 'haas' == ms ]:	
    GEN.set_style('haas')                 # use Haas style
    GEN.set_use_whitespace(True)
elif [ 'okuma' == ms ]:
    GEN.set_style('okuma')                # use Okuma style
elif [ 'heidenhain' == ms ]:
    GEN.set_style('heidenhain')           # use Heidenhain style
elif [ 'siemens' == ms ]:
    GEN.set_style('siemens')              # use Siemens style
else:
    print("invalid style chosen =",ms," valid styles are fanuc haas okuma heidenhain siemens")
    sys.exit(-1)
	
GEN.export_cncfile(DESIGN,program.nc)     # export codes

# expand export_cncfile() and print to terminal instead of file
CUR = step.Adaptive()              # walks over the process
CUR.start_project(DESIGN)
CUR.set_wanted_all()
GS = step.GenerateState()          # keeps current state of codes

GEN.set_unit_system(CUR);          # output in same units as stepnc program

# Format calls may returns 'None', so print an empty string if that happens. 
# Also, make print() omit ending newline since the strings returned by the
# format calls already have them where needed.
#
while CUR.next():
    print(GEN.format_event(GS,CUR) or , end=)