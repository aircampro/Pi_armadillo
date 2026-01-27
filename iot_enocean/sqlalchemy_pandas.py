#
# Shows how to connect to mySQL using sqlalchemy and add a record via a pandas dataframe
#
from sqlalchemy import create_engine
import pandas as pd

db_name = "kaizen"
user = "root"
passw = "rootpass"
host = "localhost"
port = 3306

con = create_engine(f'mysql+mysqlconnector://{user}:{passw}@{host}:{port}/{db_name}')
# format: 'mysql+mysqlconnector://[user]:[pass]@[host]:[port]/[schema]'
items = pd.read_sql('SELECT * FROM items;', con)
items.loc[items.id==0, 'name'] = 'syo'                                                             # changes existing user name
items = pd.concat([items, pd.DataFrame({'id': [3], 'name': ['shiro'], 'age': [28]})])              # adds record 3
items.to_sql('items', con, if_exists='upsert_overwrite', index=False)                              # upload changes to db
pd.read_sql('SELECT * FROM items;', con)                                                           # read back from db

