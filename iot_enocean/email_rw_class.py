#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# send and recieve gmails
# IMAP must be enabled to fetch them from your message box
#
#
import sys
import smtplib
from email.mime.text import MIMEText
from dotenv import load_dotenv
import os
import csv
import imaplib
import email
import time

# we are reading the email and password from the environment variables 
load_dotenv()

class GmMailer:
  """class for sending out email
  """

  def __init__(self, adr_from, pas, addr_to, subject, body):
    """GmMailer

    Args:
        addr_to (_type_): 
        subject (_type_): 
        body (_type_): 
    """

    self.password = pas
    self.addr_from = adr_from
	
    self.addr_to = addr_to
    self.subject = subject
    self.body = body
  
  def send(self):
    """ sends email
    """
    # textMIME
    msg = MIMEText(self.body.encode(self.charset), 'plain', self.charset)
    msg['Subject'] = self.subject
    msg['Form'] = self.addr_from
    msg['To'] = self.addr_to

    # gmail smtp
    # port 587
    smtp = smtplib.SMTP('smtp.gmail.com', 587)
    # EHLO、ESMTP
    smtp.ehlo()
    # TLSーSMTP
    smtp.starttls()
    smtp.ehlo()
    # SMTP
    smtp.login(self.addr_from, self.password)
    smtp.send_message(msg)
    smtp.close()

def create_mail_body(full_name, dat_in):
    """ creates a default email body template """
    body = f"""
    {full_name}
    
    This is an example of sending an email 
    
    the data read is {dat_in}。
    
    -----------------------
    Engineer
    Email: {os.getenv('EMAIL')}
    """
    return body

class GmReceiver:
    def __init__(self):
        self.imap_object = imaplib.IMAP4_SSL('imap.gmail.com')

    def login(self, email_id, password):
        try:
            self.imap_object.login(email_id,password)
            print("Login success!")
            return True
        except imaplib.IMAP4.error:
            print("Login failed!")
        return False

    def select_mailbox(self,label):
        status, response = self.imap_object.select(label)
        if 'OK' in status:
            print("{} selected!".format(label))
            return True
        else:
            print("Failed to select {} !".format(label))
            return False

    def fetch_emails(self, email_type="ALL"):
        # other options :UNSEEN, SEEN, ALL
        print("Trying to fetch {} emails ...".format(email_type.lower()))
        status, response = self.imap_object.search(None, '({})'.format(email_type))
        email_queue = []

        if 'OK' in status:
            emails = response[0].decode().split()
            if emails:
                for num in emails:
                    fetch_status, data = self.imap_object.fetch(num,'(RFC822)')
                    if 'OK' in fetch_status:
                        msg = email.message_from_string(data[0][1].decode())
                        email_queue.append(
                        {
                          "date":msg['Date'],
                          "from":msg['From'],
                          "subject":msg['Subject'],
                          "body":"".join([part.get_payload() for part in msg.walk()][1]).replace("\r","").replace("\n"," ")
                        })
            else:
                print("No vaild emails found!")

        return email_queue

    def close(self):
        print("Logging out ...",end=" ")
        self.imap_object.close()
        self.imap_object.logout()
        print("Done.")

def read_email(email_id, passwrd):
    mygmail = GmReceiver()
    if mygmail.login(email_id, passwrd):
        if mygmail.select_mailbox("INBOX"):
            emails = mygmail.fetch_emails()
            if emails:
                for mail in emails:
                    print(mail)
                    print()
        mygmail.close()

def check_from_email(email_id, passwrd, from_this_mail):
    mygmail = GmReceiver()
	c = 0
    if mygmail.login(email_id, passwrd):
        if mygmail.select_mailbox("INBOX"):
            emails = mygmail.fetch_emails()
            if emails:
                for mail in emails:
                    if not str(mail).find(from_this_mail) == -1:
                        c += 1
        mygmail.close()
    return c
	
def count_email(email_id, passwrd):
    c=0
    mygmail = GmReceiver()
    if mygmail.login(email_id, passwrd):
        if mygmail.select_mailbox("INBOX"):
            emails = mygmail.fetch_emails()
            if emails:
                for mail in emails:
                    c += 1
        mygmail.close()
    return c
	
if __name__ =="__main__":
    email_id = os.getenv('EMAIL')                                             # read from env settings 
    pass = os.getenv('PASSWORD')
	addr_to = "xx@gmail.com"
	# show how to read emails 
	tot_num_of_em = count_email(email_id, pass)
    print(f"you have {tot_num_of_em} emails")
    read_email(email_id, pass)
    # count the number of emails already in box from the reciever
	num_of_em = check_from_email(email_id, pass, addr_to)
	# now send a test email 
    subject = "Send Test e-mail"
    body = create_mail_body( "Mark Nicholas", "40.3" )
    GmMailer = GmMailer(email_id, pass, addr_to, subject, body)
    GmMailer.send()
    new_num_of_em = 0
	# now wait for confirmation email from the reciever
    while not (num_of_em == new_num_of_em):
        new_num_of_em = check_from_email(email_id, pass, addr_to) 
		time.sleep(1)
    print("completed! - we received confirmation back from receiver ",addr_to)
	
