#!/usr/bin/env python
#
# Example showing the use of steam tables from this library https://github.com/drunsinn/pyXSteam
#
from pyXSteam.XSteam import XSteam

# By using the unitSystem Parameter, you can tell XSteam witch Unit System you are using.
#
def set_units(a=0):
    if a == 0:
        steamTable = XSteam(XSteam.UNIT_SYSTEM_MKS)                    # m/kg/sec/°C/bar/W
    elif a == 1:
        steamTable = XSteam(XSteam.UNIT_SYSTEM_FLS)                    # ft/lb/sec/°F/psi/btu
    elif a == 2:
        steamTable = XSteam(XSteam.UNIT_SYSTEM_BARE)                   # m/kg/sec/K/MPa/W
    return steamTable

# example of how to call each of the functions of the library
#
def main():
    st = set_units()                                    # set the units to metric m/kg/sec/°C/bar/W
    press = 220.0                                       # bar (which would read from your instrument live)
    temp = 178.6                                        # deg Celcius
    hL_p = st.hL_p(press)                               # Saturated liquid enthalpy
    print(st.hV_p(press))	                            # Saturated vapor enthalpy
    print(st.hL_p(press))	                            # Saturated liquid enthalpy
    print(st.hV_t(temp))	                            # Saturated vapor enthalpy
    print(st.hL_t(temp))	                            # Saturated liquid enthalpy

    print(st.sV_p(press))	                            # Saturated vapor entropy
    print(st.sL_p(press))	                            # Saturated liquid entropy
    print(st.sV_t(temp))	                            # Saturated vapor entropy
    print(st.sL_t(temp))	                            # Saturated liquid entropy
    print(st.sV_p(press))	                            # Saturated vapor entropy
    sV_p = st.sV_p(press)	                            # Saturated vapor entropy
    
    print(st.tsat_p(press))	                            # Saturation temperature
    print(st.t_ph(press, hL_p))	                        # Temperature as a function of pressure and enthalpy
    print(st.t_ps(press, sV_p))	                        # Temperature as a function of pressure and entropy
    print(st.t_hs(hL_p, sV_p))	                        # Temperature as a function of enthalpy and entropy
    print(st.psat_t(temp))	                            # Saturation pressure
    print(st.p_hs(hL_p, sV_p))	                        # Pressure as a function of h and s.

    print(st.pmelt_t(temp))	                           # Pressure as a function of temperature along the melting curve. Optional parameter to select ice region
    print(st.psubl_t(temp))	                           # Pressure as a function of temperature along the sublimation curve.

    print(st.x_ph(press, hL_p))		                   # vapor fraction as a function of pressure and enthalpy
    print(st.x_ps(press, sV_p))	                       # vapor fraction as a function of pressure and entropy
    x = st.x_ph(press, hL_p)
    print(st.rhoV_p(press))	                           # Saturated vapor density
    print(st.rhoL_p(press))	                           # Saturated liquid density
    print(st.rhoV_t(temp))	                           # Saturated vapor density
    print(st.rhoL_t(temp))	                           # Saturated liquid density
    rhoV = st.rhoV_p(press)
    
    print(st.h_pt(press, temp))	                       # Enthalpy as a function of pressure and temperature
    print(st.h_ps(press, sV_p))	                       # Enthalpy as a function of pressure and entropy
    print(st.h_px(press, x))	                       # Enthalpy as a function of pressure and vapor fraction
    print(st.h_prho(press, rhoV))	                   # Enthalpy as a function of pressure and density. Observe for low temperatures (liquid) this equation has 2 solutions
    print(st.h_tx(temp, x))	                           # Enthalpy as a function of temperature and vapor fraction
    print(st.vV_p(press))		                       # Saturated vapor volume
    print(st.vL_p(press))		                       # Saturated liquid volume
    print(st.vV_t(temp))	                           # Saturated vapor volume
    print(st.vL_t(temp))	                           # Saturated liquid volume
    print(st.v_pt(press, temp))	                       # Specific volume as a function of pressure and temperature
    print(st.v_ph(press, hL_p))	                       # Specific volume as a function of pressure and enthalpy
    print(st.v_ps(press, sV_p))	                       # Specific volume as a function of pressure and entropy

    print(st.rho_pt(press, temp))	                   # Density as a function of pressure and temperature
    print(st.rho_ph(press, hL_p))	                   # Density as a function of pressure and enthalpy
    print(st.rho_ps(press, sV_p))	                   # Density as a function of pressure and entropy

    print(st.s_pt(press, temp))	                       # Specific entropy as a function of pressure and temperature (Returns saturated vapor enthalpy if mixture)
    print(st.s_ph(press, hL_p))	                       # Specific entropy as a function of pressure and enthalpy
    print(st.uV_p(press))	                           # Saturated vapor internal energy
    print(st.uL_p(press))	                           # Saturated liquid internal energy
    print(st.uV_t(temp))	                           # Saturated vapor internal energy
    print(st.uL_t(temp))	                           # Saturated liquid internal energy
    print(st.u_pt(press, temp))	                       # Specific internal energy as a function of pressure and temperature
    print(st.u_ph(press, hL_p))	                       # Specific internal energy as a function of pressure and enthalpy
    print(st.u_ps(press, sV_p))	                       # Specific internal energy as a function of pressure and entropy
    print(st.CpV_p(press))	                           # Saturated vapor heat capacity
    print(st.CpL_p(press))	                           # Saturated liquid heat capacity
    print(st.CpV_t(temp))	                           # Saturated vapor heat capacity
    print(st.CpL_t(temp))	                           # Saturated liquid heat capacity
    print(st.Cp_pt(press, temp))	                   # Specific isobaric heat capacity as a function of pressure and temperature
    print(st.Cp_ph(press, hL_p))	                   # Specific isobaric heat capacity as a function of pressure and enthalpy
    print(st.Cp_ps(press, sV_p))	                   # Specific isobaric heat capacity as a function of pressure and entropy
    print(st.CvV_p(press))	                           # Saturated vapor isochoric heat capacity
    print(st.CvL_p(press))	                           # Saturated liquid isochoric heat capacity
    print(st.CvV_t(temp))	                           # Saturated vapor isochoric heat capacity
    print(st.CvL_t(temp))	                           # Saturated liquid isochoric heat capacity
    print(st.Cv_pt(press, temp))	                   # Specific isochoric heat capacity as a function of pressure and temperature
    print(st.Cv_ph(press, hL_p))	                   # Specific isochoric heat capacity as a function of pressure and enthalpy
    print(st.Cv_ps(press, sV_p))	                   # Specific isochoric heat capacity as a function of pressure and entropy
    print(st.wV_p(press))	                           # Saturated vapor speed of sound
    print(st.wL_p(press))	                           # Saturated liquid speed of sound
    print(st.wV_t(temp))	                           # Saturated vapor speed of sound
    print(st.wL_t(temp))	                           # Saturated liquid speed of sound
    print(st.w_pt(press, temp))	                       # Speed of sound as a function of pressure and temperature
    print(st.w_ph(press, hL_p))	                       # Speed of sound as a function of pressure and enthalpy
    print(st.w_ps(press, sV_p))	                       # Speed of sound as a function of pressure and entropy
    print(st.my_pt(press, temp))	                   # Viscosity as a function of pressure and temperature
    print(st.my_ph(press, hL_p))	                   # Viscosity as a function of pressure and enthalpy
    print(st.my_ps(press, sV_p))	                   # Viscosity as a function of pressure and entropy
    print(st.tcL_p(press))	                           # Saturated vapor thermal conductivity
    print(st.tcV_p(press))	                           # Saturated liquid thermal conductivity
    print(st.tcL_t(temp))	                           # Saturated vapor thermal conductivity
    print(st.tcV_t(temp))	                           # Saturated liquid thermal conductivity
    print(st.tc_pt(press, temp))	                   # Thermal conductivity as a function of pressure and temperature
    print(st.tc_ph(press, hL_p))	                   # Thermal conductivity as a function of pressure and enthalpy
    print(st.tc_hs(hL_p, sV_p))	                       # Thermal conductivity as a function of enthalpy and entropy
    print(st.st_t(temp))	                           # Surface tension for two phase water/steam as a function of T
    print(st.st_p(press))	                           # Surface tension for two phase water/steam as a function of p

    print(st.vx_ph(press, hL_p))	                   # vapor volume fraction as a function of pressure and enthalpy
    print(st.vx_ps(press, sV_p))	                   # vapor volume fraction as a function of pressure and entropy
    print(st.pmelt_t(temp))	                           # Pressure along the melting curve as a function of temperature
    print(st.vx_psubl_tps(temp, press, sV_p))	       # Pressure along the sublimation curve as a function of temperature
    print(st.my_rhoT(rhoV, temp))	                   # Viscosity as a function of density and temperature
    print(st.tc_rhoT(rhoV, temp))	                   # Thermal conductivity as a function of density and temperature

if __name__ == "__main__":
    main()