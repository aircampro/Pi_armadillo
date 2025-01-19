#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# Example to invoke openAI chat on azure - can be used for information or museum info etc..
#
# set these env variables in your environment
# OPENAI_API_KEY=[---your api key---]
# OPENAI_RESOURCE_ENDPOINT=https://[your].openai.azure.com/ 
# OPENAI_API_TYPE=azure
# OPENAI_API_VERSION=2023-07-01-preview 
# OPENAI_DEPLOYMENT_NAME=[--your deployment name--]
#
from dotenv import load_dotenv
from os.path import join, dirname, abspath

from langchain_core.output_parsers import StrOutputParser
from langchain_core.prompts import ChatPromptTemplate
from langchain.chat_models import AzureChatOpenAI
from langchain.schema import HumanMessage
from langchain.callbacks import get_openai_callback

dir_path = dirname(abspath("__file__"))
dotenv_path = join(dir_path, '.env')
load_dotenv(dotenv_path, verbose=True)

# GPT settings
API_KEY = os.getenv("OPENAI_API_KEY")
BASE_URL = os.getenv("OPENAI_RESOURCE_ENDPOINT")
API_TYPE = os.getenv("OPENAI_API_TYPE")
API_VERSION = os.getenv("OPENAI_API_VERSION")
DEPLOYMENT_NAME = os.getenv("OPENAI_DEPLOYMENT_NAME")

system_template = "Please answer in {language}: "

user_template = """
Please answer the following question: 
{question}: """

model = AzureChatOpenAI(
    openai_api_base=BASE_URL,
    openai_api_version=API_VERSION,
    deployment_name=DEPLOYMENT_NAME,
    openai_api_key=API_KEY,
    openai_api_type=API_TYPE,
    temperature=0.5
)

model("speak a message")
parser = StrOutputParser()
prompt_template = ChatPromptTemplate.from_messages(
    [("system", system_template), ("user", user_template)]
)
chain = prompt_template | model | parser

# now you can ask the bot questions like this
chain.invoke({"language": "english", "question": "What is the highest mountain?"})

l=input("Enter language <enter = english> : ")
if l == '':
    l="english"
	
while True:
    q=input("Type your question here : ")
    chain.invoke({"language": l, "question": q})	
    a=chain.invoke({"language": l, "question": q})
    model(a)
    # alternatively
    chain.invoke()
    
	