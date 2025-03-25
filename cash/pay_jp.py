# PAY.JP Is a payment API configured based on REST.You can do a variety of things in your business operation, 
# such as paying on a case-by-case basis, making regular payments, and managing customer information.
#
# PAY.To use JP's API, register the user and obtain the API key from the API page. 
# The test key does not connect to the server that processes the production payment, 
# nor does it count as actual payment information.The production key will be available by making a production request
#
# Apple Pay
# https://pay.jp/docs/apple-pay
# Payment by credit card
# https://pay.jp/docs/mobileapp-ios
#
# Apple Pay: https://github.com/payjp/apple-pay-example
# CreditCard (Swift, Carthage): https://github.com/payjp/payjp-ios/tree/master/example-swift
# CreditCard (Objective-C, CocoaPods): https://github.com/payjp/payjp-ios/tree/master/example-objc
#
# A fee of 250 yen (tax included) when transferring sales proceeds will be borne by the member store. 
# * The transfer fee will be deducted from the sales proceeds. 
​#
# You can choose to pay once a month or twice a month.
#                  
import payjp
import sys

if (len(sys.argv)>0):
    amt=str(argv[1])
else:
    amt = "20"
    
# API key (account registered)
payjp.api_key = "API_KEY"

# User created
# It is possible to specify a unique character string of up to 100 characters for id.
# It is possible to specify a character string of up to 100 digits for id.
# up to 32 digits will be automatically generated
#
cc = payjp.Customer.create(
    id = "customerーID",
    email = "test@example.com"
)
print(cc)

# Card string
customer = payjp.Customer.retrieve("UserID")
customer.cards.create(card='Insert the card token created')

# Get card information
customer = payjp.Customer.retrieve('UserID')
card = customer.cards.retrieve('CardID')
print(card)

# make payment
charge = payjp.Charge.create(
    amount = amt,
    customer = "userID",
    card = "card ID or card token",
    currency = "JPY",
)
print(charge)