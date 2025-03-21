#!/usr/bin/env python
#             
# Example of using LINEpay driver which can be taken from here from https://qiita.com/keigohtr/items/161c9fb1584758ae3e35           
#
from flask import Flask, render_template, redirect, request
from line_pay import LinePay
from models import db, db_url, Transactions
from enum import Enum

LINE_PAY_URL = 'https://sandbox-api-pay.line.me'
LINE_PAY_CHANNEL_ID = 'your channel'
LINE_PAY_CHANNEL_SECRET = 'your secret'
LINE_PAY_CONFIRM_URL = 'http://localhost:5000/pay/confirm'

# define the vending machine sale
CHOSEN_PRODUCT="Whole Milk"
AMT = 1
CURRENCY = "JPY"

class CurrencyType(Enum):
    # LINE Pay API supports USD, JPY, TWD, THB
    USD = "USD"
    JPY = "JPY"
    TWD = "TWD"
    THB = "THB"	
    
def is_currency_not_support(cur_used):
    return CurrencyType.__members__.get(cur_used, None) == None
    
app = Flask(__name__)
pay = LinePay(channel_id=LINE_PAY_CHANNEL_ID, channel_secret=LINE_PAY_CHANNEL_SECRET, line_pay_url=LINE_PAY_URL, confirm_url=LINE_PAY_CONFIRM_URL)

@app.route("/", methods=['GET'])
def index():
    return render_template('index.html')

@app.route("/pay/reserve", methods=['POST'])
def pay_reserve():
    product_name = CHOSEN_PRODUCT
    amount = AMT
    currency = CURRENCY
    if not is_currency_not_support(currency):
        (order_id, response) = pay.request_payments(product_name=product_name, amount=amount, currency=currency)
        print(response["returnCode"])
        print(response["returnMessage"])

        transaction_id = response["info"]["transactionId"]
        print(order_id, transaction_id, product_name, amount, currency)
        obj = Transactions(transaction_id=transaction_id, order_id=order_id, product_name=product_name, amount=amount, currency=currency)
        db.session.add(obj)
        db.session.commit()
        db.session.close()
        redirect_url = response["info"]["paymentUrl"]["web"]
        return redirect(redirect_url)
    else:
        return render_template('supported_currency.html')        

@app.route("/pay/confirm", methods=['GET'])
def pay_confirm():
    transaction_id = request.args.get('transactionId')
    obj = Transactions.query.filter_by(transaction_id=transaction_id).one_or_none()
    if obj is None:
        raise Exception("Error: transaction_id not found.")

    response = pay.confirm_payments(transaction_id=transaction_id, amount=obj.amount, currency=obj.currency)
    print(response["returnCode"])
    print(response["returnMessage"])

    db.session.query(Transactions).filter(Transactions.transaction_id == transaction_id).delete()
    db.session.commit()
    db.session.close()
    return "Payment successfully finished."

@app.route("/pay/refund", methods=['GET'])
def pay_refund():
    transaction_id = request.args.get('transactionId')
    obj = Transactions.query.filter_by(transaction_id=transaction_id).one_or_none()
    if obj is None:
        raise Exception("Error: transaction_id not found.")

    response = pay.refund_payments(transaction_id=transaction_id, amount=obj.amount)
    print(response["returnCode"])
    print(response["returnMessage"])

    db.session.query(Transactions).filter(Transactions.transaction_id == transaction_id).delete()
    db.session.commit()
    db.session.close()
    return "Refund successfully finished."
    
def initialize_app(app: Flask) -> None:
    app.config['SQLALCHEMY_DATABASE_URI'] = db_url
    app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = True
    app.config['DEBUG'] = True
    db.init_app(app)
    db.create_all(app=app)

def main() -> None:
    initialize_app(app)
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    main()
    