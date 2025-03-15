#!/usr/bin/env python
#
# this example is taken from here https://adrpy.readthedocs.io/en/latest/
# Given a climb rate (in feet per minute) and a climb speed (KIAS), as well as an altitude 
# (in a given atmosphere) where these must be achieved, as well as a set of basic geometrical 
# and aerodynamic performance parameters, compute the necessary T/W ratio to hold the specified climb rate.
#
from ADRpy import atmospheres as at
from ADRpy import constraintanalysis as ca

def calc1():
    designbrief = {'climbalt_m': 0, 'climbspeed_kias': 101, 'climbrate_fpm': 1398}

    etap = {'climb': 0.8}

    designperformance = {'CDminclean': 0.0254, 'etaprop' :etap}

    designdef = {'aspectratio': 10.12, 'sweep_le_deg': 2, 'sweep_mt_deg': 0, 'bpr': -1}

    TOW_kg = 2042.0

    designatm = at.Atmosphere()

    concept = ca.AircraftConcept(designbrief, designdef, designperformance, designatm)

    # float or array, list of wing-loading values in Pa.
    wingloadinglist_pa = [1250, 1500, 1750, 2000]                            

    twratio = concept.twrequired_clm(wingloadinglist_pa)

    print('T/W: ', twratio)
	return twratio

# Given a load factor, an altitude (in a given atmosphere) and a true airspeed, 
# as well as a set of basic geometrical and aerodynamic performance parameters, compute the necessary T/W ratio 
# to hold that load factor in the turn.
#
from ADRpy import unitconversions as co

def calc1():
    designbrief = {'stloadfactor': 2, 'turnalt_m': co.feet2m(10000), 'turnspeed_ktas': 140}

    etap = {'turn': 0.85}

    designperformance = {'CLmaxclean': 1.45, 'CDminclean':0.02541, 'etaprop': etap}

    designdef = {'aspectratio': 10.12, 'sweep_le_deg': 2, 'sweep_mt_deg': 0, 'bpr': -1}

    designatm = at.Atmosphere()

    concept = ca.AircraftConcept(designbrief, designdef, designperformance, designatm)

    # float or array, list of wing-loading values in Pa.
    wingloadinglist_pa = [1103, 1250, 1500, 1750, 1920]

    twratio, clrequired, feasibletw = concept.twrequired_trn(wingloadinglist_pa)

    print('T/W:               ', twratio)
    print('Only feasible T/Ws:', feasibletw)
    print('CL required:       ', clrequired)
    print('CLmax clean:       ', designperformance['CLmaxclean'])
    return twratio, feasibletw, clrequired, designperformance['CLmaxclean']
	
if __name__ == "__main__": 	

    a = calc1()
    b, c, d, e = calc2()	
