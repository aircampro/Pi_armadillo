# pip install stripe
#
# library for using strike payment system e.g. automated shop you can even create monthy accounts and print a years QR codes for machine use
#
import stripe

stripe.api_key = "sk_test_4eC39HqLyjWDarjtT1zdp7dc"                                           # "STRIPE_SECRET_KEY"

#creation
def create_strike_cust(nme="your_name",em="has@noemail.com"):
    stripe_customer = stripe.Customer.create(name=nme,email=em)
    return stripe_customer, stripe_customer.id
	
# aquire e.g. id="cus_XXXXX"
def get_customer(id="stripe_customer_id"):
    customer = stripe.Customer.retrieve(id)
    return customer

# update customer email
def update_email(nme="your_name",em="has@noemail.com"):
    customer = stripe.Customer.modify(id, email=em)
    return customer
	
# delete
def del_customer(id="stripe_customer_id"):
    stripe.Customer.delete(id)

# create payment method for card this return the card_id
def attach_card(no="4242424242424242", em=12, ey=2027, cy="824"):
    payment_method = stripe.PaymentMethod.create( type="card",  card={ "number": no, "exp_month": em, "exp_year": ey, "cvc": cy })	
    return payment_method

def attach_pm(pms="pm_1FkUxe2eZvKYlo2C0JAaEdWV", id="cus_XXXXX"):
    payment_method_attached = stripe.PaymentMethod.attach( pms, customer=id )
    return payment_method_attached
 	
# list
def get_list(id="stripe_customer_id"):
    cards = stripe.PaymentMethod.list( customer=id, type="card",)
    return cards

# get last card in multiple list	
def get_latest_card(crd):
    if cards.has_more:
        latest_id = cards.data[-1].id
    return latest_id

# charge amex card
def amex_pay(amount=2000, cur="usd", d="My First Test Charge"):
    charge = stripe.Charge.create(
        amount=amount,  
        currency=cur,
        source="tok_amex",  
        description=d
    )
    return charge

# this shows the above with error handling you can implement with all calls if you need
def amex_pay_errorH(amount=2000, cur="usd", d="My First Test Charge"):
    try:
        charge = stripe.Charge.create(
            amount=amount,
            currency=cur,
            source="tok_amex",
            description=d
        )
    except stripe.error.CardError as e:
        body = e.json_body
        err  = body.get('error', {})
        print(f"Status is: {e.http_status}")
        print(f"Type is: {err.get('type')}")
        print(f"Code is: {err.get('code')}")
        print(f"Message is: {err.get('message')}")
    except stripe.error.RateLimitError as e:
        pass
    except stripe.error.InvalidRequestError as e:
        pass
    except stripe.error.AuthenticationError as e:
        pass
    except stripe.error.APIConnectionError as e:
        pass
    except stripe.error.StripeError as e:
        pass
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    
# get paid
#
def retrieve_charge(charg="ch_XXXXX")
    charge = stripe.Charge.retrieve(charg)	
    return charge

# create and pay invoices
def create_invoice(id="cus_XXXXX"):
    invoice = stripe.Invoice.create(customer=id,  auto_advance=True)
    return invoice

def pay_invoice(iid="in_XXXXX"):
    invoice = stripe.Invoice.pay(iid)	
    return invoice
	
# Process payments using customers and payment methods	
# make a payment using card_id returned when you called attach_card
#
def make_pay(amount = 1000, cur="jpy", cid="card_id", cus="stripe_customer_id"):
    pi = stripe.PaymentIntent.create(
        customer=cus,
        amount=amount,
        currency=cur,
        payment_method=cid,
        confirm=True,
        automatic_payment_methods={"enabled": True, "allow_redirects": "never"},
        metadata={"project_id": "project_id"},
    )
    return pi
	
def confirm_pay(payment_method_ID="pi_XXXXX"):
    payment_intent = stripe.PaymentIntent.confirm(payment_method_ID, payment_method=payment_method_ID)
    return payment_intent

def refund_charge(chg="ch_XXXXX"):
    refund = stripe.Refund.create(charge=chg)	
    return refund

# creating products and attaching prices to them
def create_product(nm="Whole Milk", t="Milk"):
    product = stripe.Product.create(name=nm,  type=t)	
    return product

def price_to_product(pn="prod_XXXXX", a=2000, c="usd"):
    price = stripe.Price.create( unit_amount=a, currency=c, product=pn)
	
def monthly_price_to_product(pn="prod_XXXXX",a=2000, c="usd"):
    price = stripe.Price.create( unit_amount=a, currency=c, recurring={"interval": "month"},  product=pn)	

def verify_pm(a=[32, 45],pm="pm_XXXXX"):
    stripe.PaymentMethod.verify(pm,  amounts=a)	

# Create an account (for Connect)	
def cre_acc():
    account = stripe.Account.create( type="standard")
    return account	
	
	
	
