#!/usr/bin/env python3
# This example shows http webpage automation using splinter also demonstrates neat color writer class
#
# this can be automating work on the internet or controlling robot/plant/machine via web browser interface
#
from splinter import Browser

class Bcolors:
    HEADER = '\033[95m'
    CYAN = "\033[36m"
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    RESET = '\033[0m'
    REVERSE = "\033[;7m"

example_browsers = ['chrome', 'firefox', 'phantomjs']

def automated_login(url, brow=example_browsers[2], username, password):
    with Browser(brow) as browser:
        browser.visit(url)
        browser.find_by_id('username').fill(username)
        browser.find_by_css('#password input').fill(password)
        browser.find_by_value('OK').click()

def pics_list(url='http://www.yahoo.co.jp/', brow=example_browsers[0]):
    with Browser(brow) as browser:
        browser.visit(url)
        div = browser.find_by_id('gridlist').first
        browser.find_by_css('#password input').fill(password)
        browser.find_by_value('OK').click()
        for img in div.find_by_tag('img'):
            print img['src']

if __name__ == "__main__":
    automated_login(url="www.yoursite.com", brow=example_browsers[1], username="you", password="pass")       # login with firefox
    print("{}[-] Website logged {}".format(Bcolors.GREEN, Bcolors.RESET))                                    # green
    print("{}{}".format(Bcolors.YELLOW, Bcolors.REVERSE))                                                    # yellow reverse video 
    print("{}[-] picture files {}".format(Bcolors.YELLOW, Bcolors.RESET))                                    # write and reset   
    pics_list()                                                                                              # list pics on yahoo



