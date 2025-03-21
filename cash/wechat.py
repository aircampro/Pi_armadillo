#!/usr/bin/env python
#
# Example using WeChat Pay has been adapted from the API it uses from here https://github.com/Yucheng-Ren/wechat-pay
#
from wechatpay import WeChatPay
import webbrowser
import pickle 
import os

WECHAT_APPID = 'your_app_id'
WECHAT_MCH_ID = 'your_mch_id'
WECHAT_NOTIFY_URL = 'your_notify_url'
WECHAT_PAY_SECRET = 'your_pay_secret'
WECHAT_CERT = 'path/to/your_cert.pem'
WECHAT_KEY = 'patch/to/your_key.pem'

# stores the order sequence number which increments
class OrderNumberSequence():
    def __init__(self, i_state=0, ip="10.0.12.1", msg="vending machine order", cost=10.0):
        print("Inititialised Order Number Sequence class")
        self.order_number = i_state
        self.file_nm = "wechatseq.pkl"	
        self.ip = ip  
        self.bdy = msg 
        self.charge = cost         
    def save_pickle_file(self, o):
        with open(self.file_nm, 'wb') as f:
            pickle.dump(o, f) 
    def new_order(self):
        self.order_number += 1
    def change_cost(self,val):
        self.charge = val
    def change_body(self, msg):
        self.bdy = msg 
    def change_ip(self, ip):
        self.ip = ip
        
# loads the order number from the pickle file or initiates the class with the order begining at the variable specified
def make_sequencer_class(seq_start=0):
    if os.path.exists("wechatseq.pkl"):		
        with open("wechatseq.pkl", 'rb') as f:
            r = pickle.load(f) 
    else:
        r = OrderNumberSequence(seq_start)
    return r
	
sp = WeChatPay(WECHAT_APPID, WECHAT_MCH_ID, WECHAT_NOTIFY_URL, WECHAT_PAY_SECRET, WECHAT_CERT, WECHAT_KEY)

# create a new order
#
def unifiedorder(out_trade_no, body, total_fee, client_ip):
    '''
    :param out_trade_no: your order id, should be unique in your system
    :param body: order description
    :param total_fee: order price
    :param client_ip: request ip address
    :return:
    '''
    result = sp.unifiedorder(body=body, out_trade_no=out_trade_no, total_fee=total_fee, spbill_create_ip=client_ip)
    if result.success:
        # Qr code url
        return result.code_url, 0
    else:
        return result.error_msg, -1

def query_order(out_trade_no):
    result = sp.query_order(out_trade_no=out_trade_no)
    if result.success:
        order_state = result.trade_state
        return order_state
    else:
        return result.error_msg
		
def show_url(url):
    webbrowser.open(url, new=0, autoraise=True)
	
if __name__ == '__main__':

    sno = make_sequencer_class()
    sno.new_order()
	sno.save_pickle_file(sno) 	
    url = unifiedorder(sno.order_number, sno.bdy, sno.charge, sno.ip)
    show_url(url)