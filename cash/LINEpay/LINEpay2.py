#!/usr/bin/env python
#
# LINE pay Example apapted from here... with HMAC Signature and X-Line Auth
# https://qiita.com/maztak/items/39d46f0d83f895e626c8
# https://github.com/sumihiro3/line-pay-sdk-python/tree/master/examples
#
# -*- coding: utf-8 -*-
#
"""
LINE Pay API SDK for Python use example

Request -> Confirm -> PreApproved -> Capture
"""

import logging
import uuid
import os
from os.path import join, dirname
from dotenv import load_dotenv
from flask import Flask, request, render_template
from linepay import LinePayApi
import sys

# set your currency here
CURRENCY_NOTE = "USD"

# dotenv
load_dotenv(verbose=True)
dotenv_path = join(dirname(__file__), '.env')
load_dotenv(dotenv_path)

# logger
logger = logging.getLogger("linepay")
logger.setLevel(logging.DEBUG)
sh = logging.StreamHandler()
logger.addHandler(sh)
formatter = logging.Formatter('%(asctime)s:%(lineno)d:%(levelname)s:%(message)s')
sh.setFormatter(formatter)

# Flask webserver
app = Flask(__name__)

# LINE Pay API gets them from your environament variables create the object to communicate on REST API with the LINEpay system
LINE_PAY_CHANNEL_ID = os.environ.get("LINE_PAY_CHANNEL_ID")
LINE_PAY_CHANNEL_SECRET = os.environ.get("LINE_PAY_CHANNEL_SECRET")
LINE_PAY_REQEST_BASE_URL = "https://{}".format(
    # set your server host name (ex. ngrok forwarding host) at HOST_NAME on .env file
    os.environ.get("HOST_NAME")
)
api = LinePayApi(LINE_PAY_CHANNEL_ID, LINE_PAY_CHANNEL_SECRET, is_sandbox=True)

# check if the chosen currency is supported
if api.is_supported_currency(CURRENCY_NOTE) is False:
    print("currency[{}] is not supported".format(CURRENCY_NOTE))
    sys.exit(-1)
    
# Cache
CACHE = {}

@app.route("/request_order_milk", methods=['GET'])
def pay_request():
    order_id = str(uuid.uuid4())
    amount = 1
    currency = CURRENCY_NOTE
    product_name = "Fresh Whole Milk"
    CACHE["order_id"] = order_id
    CACHE["amount"] = amount
    CACHE["currency"] = currency
    CACHE["product_name"] = product_name
    request_options = {
        "amount": amount,
        "currency": currency,
        "orderId": order_id,
        "packages": [
            {
                "id": "package-999",
                "amount": 1,
                "name": "Sample package",
                "products": [
                    {
                        "id": "WM001-A",
                        "name": product_name,
                        "imageUrl": "https://sinagra-family-dairy.odoo.com/web/image/1405-b8157174/sinagra-raw-milk.webp",
                                    "quantity": 1,
                                    "price": amount
                    }
                ]
            }
        ],
        "options": {
            "payment": {
                "payType": "PREAPPROVED"
            }
        },
        "redirectUrls": {
            "confirmUrl": LINE_PAY_REQEST_BASE_URL + "/confirm",
            "cancelUrl": LINE_PAY_REQEST_BASE_URL + "/cancel"
        }
    }
    logger.debug(request_options)
    response = api.request(request_options)
    logger.debug(response)
    # Check Payment Satus
    transaction_id = int(response.get("info", {}).get("transactionId", 0))
    check_result = api.check_payment_status(transaction_id)
    logger.debug(check_result)
    response["transaction_id"] = transaction_id
    response["paymentStatusCheckReturnCode"] = check_result.get("returnCode", None)
    response["paymentStatusCheckReturnMessage"] = check_result.get("returnMessage", None)
    return render_template("request.html", result=response)

@app.route("/request_order_choco", methods=['GET'])
def pay_request2():
    order_id = str(uuid.uuid4())
    amount = 1.5
    currency = CURRENCY_NOTE
    product_name = "Chocolate Milk"
    CACHE["order_id"] = order_id
    CACHE["amount"] = amount
    CACHE["currency"] = currency
    CACHE["product_name"] = product_name
    request_options = {
        "amount": amount,
        "currency": currency,
        "orderId": order_id,
        "packages": [
            {
                "id": "package-999",
                "amount": 1,
                "name": "Sample package",
                "products": [
                    {
                        "id": "OJ001-A",
                        "name": product_name,
                        "imageUrl": "https://www.creamoland.com/img/COLproducts/40930.jpg",
                                    "quantity": 1,
                                    "price": amount
                    }
                ]
            }
        ],
        "options": {
            "payment": {
                "payType": "PREAPPROVED"
            }
        },
        "redirectUrls": {
            "confirmUrl": LINE_PAY_REQEST_BASE_URL + "/confirm",
            "cancelUrl": LINE_PAY_REQEST_BASE_URL + "/cancel"
        }
    }
    logger.debug(request_options)
    response = api.request(request_options)
    logger.debug(response)
    # Check Payment Status
    transaction_id = int(response.get("info", {}).get("transactionId", 0))
    check_result = api.check_payment_status(transaction_id)
    logger.debug(check_result)
    response["transaction_id"] = transaction_id
    response["paymentStatusCheckReturnCode"] = check_result.get("returnCode", None)
    response["paymentStatusCheckReturnMessage"] = check_result.get("returnMessage", None)
    return render_template("request.html", result=response)

@app.route("/confirm", methods=['GET'])
def pay_confirm():
    transaction_id = int(request.args.get('transactionId'))
    logger.debug("transaction_id: %s", str(transaction_id))
    CACHE["transaction_id"] = transaction_id
    response = api.confirm(
        transaction_id,
        float(CACHE.get("amount", 0)),
        CACHE.get("currency", CURRENCY_NOTE)
    )
    logger.debug(response)
    reg_key = response.get("info", {}).get("regKey", None)
    CACHE["reg_key"] = reg_key
    # Check RegKey
    check_result = api.check_regkey(reg_key)
    logger.debug(check_result)
    response["regKeyCheckReturnCode"] = check_result.get("returnCode", None)
    response["regKeyCheckReturnMessage"] = check_result.get("returnMessage", None)
    return render_template("confirm-preapproved.html", result=response)

@app.route("/pay_preapproved", methods=['GET'])
def pay_preapproved():
    reg_key = CACHE.get("reg_key", None)
    product_name = CACHE.get("product_name", None)
    amount = float(CACHE.get("amount", 0))
    currency = CACHE.get("currency", CURRENCY_NOTE)
    order_id = str(uuid.uuid4())
    logger.debug("reg_key: %s", str(reg_key))
    response = api.pay_preapproved(reg_key, product_name, amount, currency, order_id, capture=False)
    logger.debug(response)
    # update transaction_id for pre-approved authorized transaction
    transaction_id = int(response.get("info", {}).get("transactionId", 0))
    CACHE["transaction_id"] = transaction_id
    # Check RegKey
    check_result = api.check_regkey(reg_key)
    logger.debug(check_result)
    response["regKeyCheckReturnCode"] = check_result.get("returnCode", None)
    response["regKeyCheckReturnMessage"] = check_result.get(
        "returnMessage", None)
    return render_template("capture-preapproved.html", result=response)

@app.route("/capture", methods=['GET'])
def pay_capture():
    transaction_id = int(CACHE.get("transaction_id", 0))
    amount = float(CACHE.get("amount", 0))
    currency = CACHE.get("currency", None)
    logger.debug("transaction_id: %s", str(transaction_id))
    logger.debug("amount: %s", str(amount))
    logger.debug("currency: %s", str(currency))
    response = api.capture(transaction_id, amount, currency)
    logger.debug(response)
    return response

if __name__ == "__main__":
    app.run(debug=True, port=8000)
	