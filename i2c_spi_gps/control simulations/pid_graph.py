#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# PID Simulator ref:- https://qiita.com/akinami/items/29a851c7051b1968b396
#
#
import matplotlib.pyplot as plt
import ipywidgets

# ------------------------------- create GUI ------------------------------
# text widgets
def generate_vbox_text_widget():
    text_widgets = []
    text_widgets.append(ipywidgets.FloatText(min=0.0, max=359.0))                        # theta_start
    text_widgets.append(ipywidgets.FloatText(min=0.0, max=359.0))                        # theta_goal
    text_widgets.append(ipywidgets.FloatText(min=0.0, max=100.0))                        # offset
    text_widgets.append(ipywidgets.IntText(min=-360, max=360))                           # time_length
    text_widgets.append(ipywidgets.FloatText(min=0.00, max=1.50))                        # kp
    text_widgets.append(ipywidgets.FloatText(min=0.00, max=1.50))                        # ki
    text_widgets.append(ipywidgets.FloatText(min=0.00, max=1.50))                        # kd
    text_widgets.append(ipywidgets.FloatText(min=0.00, max=1.0))                         # action forward or reverse
    vox_text_widgets = ipywidgets.VBox(text_widgets)
    return vox_text_widgets

# slider widgets
def generate_vbox_slider_widget():
    slider_widgets = []
    slider_widgets.append(ipywidgets.FloatSlider(value=0.0, min=0.0, max=359.0, description = "theta_start", disabled=False))
    slider_widgets.append(ipywidgets.FloatSlider(value=90.0, min=0.0, max=359.0, description = "theta_goal", disabled=False))
    slider_widgets.append(ipywidgets.FloatSlider(value=0.0, min=0.0, max=100.0, step=0.01, description = "offset", disabled=False))
    slider_widgets.append(ipywidgets.IntSlider(value=150, min=0, max=2000, description = "time_length", disabled=False))
    slider_widgets.append(ipywidgets.FloatSlider(value=0.10, min=0.00, max=1.50, step=0.001, description = "kp", disabled=False))
    slider_widgets.append(ipywidgets.FloatSlider(value=0.50, min=0.00, max=1.50, step=0.001, description = "ki", disabled=False))
    slider_widgets.append(ipywidgets.FloatSlider(value=0.50, min=0.00, max=1.50, step=0.001, description = "kd", disabled=False))
    slider_widgets.append(ipywidgets.FloatSlider(value=0.0, min=0.00, max=1.0, step=1.0, description = "reverse", disabled=False))
    vox_slider_widgets = ipywidgets.VBox(slider_widgets)
    return vox_slider_widgets

# Box
def link_slider_and_text(box1, box2):
    for i in range(7):
      ipywidgets.link((box1.children[i], 'value'), (box2.children[i], 'value'))

# Main window this calls main below with the parameters
def draw_interactive():
    # slider widget
    sliders = generate_vbox_slider_widget()
    # text widget
    texts = generate_vbox_text_widget()

    # slider widget &　posture widget 
    slider_and_text = ipywidgets.Box([sliders, texts])

    # slider wiget & text widget 
    link_slider_and_text(sliders, texts)

    # main & slider widgets
    params = {}
    for i in range(7):
        params[str(i)] = sliders.children[i]
    final_widgets = ipywidgets.interactive_output(main, params)
    
    display(slider_and_text, final_widgets)

# -------------------------------------- PID ----------------------------------------
def PID(kp, ki, kd, theta_goal, theta_current, error_sum, error_pre):
    error = theta_goal - theta_current                     # instant error
    error_sum += error                                     # integrated error
    error_diff = error-error_pre                           # difference error
    m = (kp * error) + (ki * error_sum) + (kd*error_diff)  # calculate factor
    return m, error_sum, error

# main action does the PID and displays it
def main(*args, **kwargs):

    params = kwargs
    theta_start = params["0"]                                                                 # read from the GUI
    theta_goal = params["1"]
    offset = params["2"]
    time_length = params["3"]
    kp = params["4"]
    ki = params["5"]
    kd = params["6"]
    rev = params["7"]
    
    error_sum = 0.0
    error_pre = 0.0
    theta_current = theta_start
    time_list = [0]
    theta_list = [theta_start]

    # PID -----------------------
    for time in range(1, time_length):
        m, error_sum, error = PID(kp, ki, kd, theta_goal, theta_current, error_sum, error_pre) # make PID Class object
        if rev == 0.0:
            theta_current += m                                                                 # Add the manipulated variable to the current angle (actually, the motor is moved based on this manipulated variable)
            theta_current -= offset
        else:
            theta_current -= m                                                               
            theta_current += offset        
        error_pre = error                                                                      # save previous error
        time_list.append(time)                                                                 # create time list
        theta_list.append(theta_current)                                                       # create theta list

    # GUI
    plt.hlines([theta_goal], 0, time_length, "red", linestyles='dashed')                       # goal setpoint
    plt.plot(time_list, theta_list, label="PID", color="blue")                                 # PID values
    plt.xlabel(r'$t$')                                                                         # x axis is t 
    plt.ylabel(r'$\theta$')                                                                    # y axis is a theta symbol
    plt.ylim(theta_start-20, theta_goal+60)                                                    
    plt.legend(loc='lower right')                                                              # locate legend
    plt.title(r'final $\theta$={:.3g}'.format(theta_list[-1]))                                 # write title
    plt.show()                                                                                 # show plot

if __name__ == "__main__": 
    # draw the simulation window
    draw_interactive()