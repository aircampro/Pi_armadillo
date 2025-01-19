#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# Example program to be used with android icloud service via phone registration. This program runs on razpi using pyicloud 
# use icloud service to obtain location information, then email it using gmail, also makes a map location and emails it
#
# to work on razpi do following
# https://github.com/picklepete/pyicloud
# https://qiita.com/azipinsyan/items/1f70bc66c2ff23a0a411
# pi@rasberry ~/work/$ python setup.py build
# pi@rasberry ~/work/$ python setup.py install
# After the installation is complete, you can get latitude and longitude information from iCloud's "Find My iPhone" in Python.
# pip install pyicloud
#
import folium
from pyicloud import PyiCloudService
from getpass import getpass

import smtplib
from email.mime.text import MIMEText
# pip3 install --upgrade reverse_geocoder
#
import get_gps, reverse_geocoder as rg

from geopy.distance import geodesic

# simple send gmail message
#
def send_gmail(sender_email, sender_password, recipient_email, subject, body):    
    smtp_server = 'smtp.gmail.com'  
    smtp_port = 587  

    msg = MIMEText(body)
    msg['Subject'] = subject
    msg['From'] = sender_email
    msg['To'] = recipient_email

    try:
        server = smtplib.SMTP(smtp_server, smtp_port)
        server.ehlo()  
        server.starttls()
        server.login(sender_email, sender_password)
        server.sendmail(sender_email, recipient_email, msg.as_string())
        server.quit()
        print("sent email with location information to ", recipient_email)
    except Exception as e:
        print("error in sending : ", e)

# library of email functions (for more complex emails)
#
import os
import smtplib
import ssl
import sys
import glob
from email import encoders
from email.mime.base import MIMEBase
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from pathlib import Path
import datetime

def sendgmail_ssl_attach(sender_email, sender_password, sendto, attachF):
    account = sender_email
    password = sender_password
    smtp_server = 'smtp.gmail.com' 
    smtp_port = 465
	
    to_email = sendto
    from_email = sender_email

    subject = "sending file attachment"
    d=datetime.datetime.now()
    message = "this is the file atatched\n"+str(d)
    msg = MIMEMultipart()
    msg["Subject"] = subject
    msg["To"] = to_email
    msg["From"] = from_email
    msg.attach(MIMEText(message))
	
    for file in attachF:
        fname = os.path.basename(file)
        part = MIMEBase('application', 'octet-stream')
        part.set_payload(open(file, 'rb').read())
        encoders.encode_base64(part)
        part.add_header('Content-Disposition', 'attachment', filename=fname)
        msg.attach(part)

    try:
        server = smtplib.SMTP_SSL(smtp_server, smtp_port, context=ssl.create_default_context())
        server.login(account, password)
        server.send_message(msg)
        server.quit()
        print("sent email to ", recipient_email)
    except Exception as e:
        print("error in sending : ", e)
        
def sendgmail_tls_attach_with_cc(sender_email, sender_password, sendto, cc_email, filenm):

    to_email = sendto
    from_email = sender_email
    cc_mail = cc_email

    subject = f"attached file = {filenm}"
    d=str(datetime.datetime.now())
    message = f"this is the file atatched\n {d}"
    msg = MIMEMultipart()
    msg["Subject"] = subject
    msg["To"] = to_email
    msg["From"] = from_email
    msg['cc'] = cc_mail
    msg.attach(MIMEText(message))

    filepath= filenm
    filename = os.path.basename(filepath)
    with open(filepath, 'rb') as f:
        attach = MIMEApplication(f.read())   
    attach.add_header('Content-Disposition', 'attachment', filename=filename)
    msg.attach(attach)
	
    smtp_host = 'smtp.gmail.com'
    smtp_port = 587
    smtp_password = sender_password

    try:
        server = smtplib.SMTP(smtp_host, smtp_port)
        server.ehlo()
        # TLS
        server.starttls()
        # SMTP login
        server.login(from_email, smtp_password)
        server.send_message(msg)
        server.quit()
        print("sent email to ", recipient_email)
    except Exception as e:
        print("error in sending : ", e)
        
def sendmail_html_with_cc(sender_email, sender_password, sendto, cc_email):

    to_email = sendto
    from_email = sender_email
    cc_mail = cc_email

    msg = MIMEMultipart('alternative')
    msg["To"] = to_email
    msg["From"] = from_email
    msg['cc'] = cc_mail
    d=datetime.datetime.now()
    text = " This is an html message "
    html = """
    <html>
      <head></head>
      <body>
        <p style='font-size:16.0pt;font-family:Testing Html Email</p>
        <p>The style of this email is now html</p>
        <p>hope thats useful and you can replace with your own including images</p>
        <p>this example is not using ssl or tls to the server which is your domain</p>
        <p>Here I'm sending this when your at the specified location</p>
      </body>
    </html>
    """
    part1 = MIMEText(text, 'plain')
    part2 = MIMEText(html, 'html')
    msg.attach(part1)
    msg.attach(part2)

    smtp_host = 'smtp.xxx.xx.xx'                                            # e.g. own domain that requires no tls or ssl
    smtp_port = 587
    smtp_password = sender_password

    try:	
        server = smtplib.SMTP(smtp_host, smtp_port, timeout=10)
        server.login(from_email, smtp_password)
        server.sendmail(from_mail, to_mail, msg.as_string())
        server.quit()
        print("sent email to ", recipient_email)
    except Exception as e:
        print("error in sending : ", e)
        
def sendgmail_tls_html_attach_with_cc(sender_email, sender_password, sendto, cc_email, filenm):

    to_email = sendto
    from_email = sender_email
    cc_mail = cc_email

    msg = MIMEMultipart('alternative')
    msg["To"] = to_email
    msg["From"] = from_email
    msg['cc'] = cc_mail
    d=datetime.datetime.now()
    text = "this is the location map on time "+str(d)+" for "+str(sender_email)
    filepath= filenm
    filename = os.path.basename(filepath)
    with open(filepath, 'rb') as f:
        html = f.read() 
    part1 = MIMEText(text, 'plain')
    part2 = MIMEText(html, 'html')
    msg.attach(part1)
    msg.attach(part2)

    smtp_host = 'smtp.gmail.com'
    smtp_port = 587
    smtp_password = sender_password

    try:	
        server = smtplib.SMTP(smtp_host, smtp_port, timeout=10)
        server.ehlo()
        # TLS
        server.starttls()
        server.login(from_email, smtp_password)
        server.sendmail(from_mail, to_mail, msg.as_string())
        server.quit()
        print("sent email to ", recipient_email)
    except Exception as e:
        print("error in sending : ", e)
        
def wrap_map_to_send():	
    return glob.glob(r"map.html")		

# ------------------- global -------------------------------

# location to look for use this to find your locatrion
# https://www.tree-maps.com/click-and-get-coordinate/
# 
location_alert = (55.718726184959635, -2.427454068731587)              # if we reach here we send a html message alert type email 
radius_threshold = 0.03                                                # 30m
   
# ask user for account credentials e.g. xxx@icloud.com
iCloudAccount = getpass('Enter your icloud Account: ')
iCloudPassword = getpass('Enter your icloud Password: ')
api = PyiCloudService(iCloudAccount, iCloudPassword)

# first time you use the service you need this verify procedure with your mobile phone
if api.requires_2fa:
    import click
    print("Two-factor authentication required. Your trusted devices are:")

    devices = api.trusted_devices
    for i, device in enumerate(devices):
        print("  %s: %s" % (i, device.get('deviceName', "SMS to %s" % device.get('phoneNumber'))))

    device = click.prompt('Which device would you like to use?', default=0)
    device = devices[device]
    if not api.send_verification_code(device):
        print("Failed to send verification code")
        sys.exit(1)

    code = click.prompt('Please enter validation code')
    if not api.validate_verification_code(device, code):
        print("Failed to verify verification code")
        sys.exit(1)

# gets gps location information from icloud service
def get_oauth():
    auth = api.devices[0].location()
    return auth

if __name__ == '__main__':
    sender_email = 'your_email@gmail.com'
    sender_password = 'your_password'
    recipient_email = 'breakdownservice@rac.com'
    # get gps position from iCloud
    auth = str(get_oauth())
    s = auth.find('longitude')
    e = auth.find(u'positionType')
    s1 = auth.find(u'latitude')
    e1 = auth.find(u'isOld')
    lng1 = float(auth[s+12:e-4])
    lat1 = float(auth[s1+11:e1-4])
    print('%3.14f,%3.14f'%(lng1,lat1))
    subject = 'my current location'
    results = rg.search([(lat1, lng1)])
    area = {t: results[0][t] for t in results[0]}
    body = 'the current location is lat='+str(lat1)+" lon="+str(lng1)+" which is "+area['cc']+" "+area['admin1']+" "+area['name']
    # send a text email
    send_gmail(sender_email, sender_password, recipient_email, subject, body)	
    # make a html map of the location and pin-point it 
    mymap = folium.Map(location=[lat1, lng1], zoom_start=15, tiles='openstreetmap')
    folium.Marker(
        location=[lat1, lng1],
        popup="<i>current position</i>",  
        tooltip="iam here",  
        icon=folium.Icon(color="red", icon="home")  
    ).add_to(mymap)
    mymap.save("map.htnl")
    snd_data = wrap_map_to_send()
    # send as attachment this time from ssl mailbox
    sender_email2 = 'your_ssl_email@gmail.com'
    sendgmail_ssl_attach(sender_email2, sender_password, recipient_email, snd_data) 
    # send as an html email the map with a return email copy as the receipt for sending
    sendgmail_tls_html_attach_with_cc(sender_email, sender_password, recipient_email, sender_email, "map.html")
    # find distance to alert position if its inside then send the alert 
    distance = geodesic(current_coordinate, convenience_store).kilometers
    if distance <= radius_threshold:
        sender_email3 = aa@bb.com.br
        sendmail_html_with_cc(sender_email3, sender_password, recipient_email, sender_email)    
    sys.exit(0)
    