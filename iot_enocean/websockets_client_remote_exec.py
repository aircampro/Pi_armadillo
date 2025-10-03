#!/usr/bin/env python
# this is a websocket cleint for remote commands execution
#
import asyncio
import logging
import time
import argparse

from websockets import connect

class WebsocketConnector:
    def __init__(self, ip_address, port):
        """
        This class is used to create an instance of WebSockerConnector, which can be used to connect to a remote machine
        and then execute commands on remote machine.

        :param str ip_address: the remote machine's ip_address.
        :param str port: the port number on which websocket is running on server machine.
        """
        self.ip_address = ip_address
        self.URL = f"ws://{self.ip_address}:{port}"
        self.conn = None
        self.loop = asyncio.get_event_loop()

    def get_connection(self):
        conn = None
        retry_count = 10
        while conn is None and retry_count > 0:
            try:
                logging.debug(f"Establishing connection with {self.URL}")
                conn = connect(self.URL)
            except Exception as ex:
                logging.warning(f"Failed to establish connection.. retrying again after 1 min. {ex}")
                time.sleep(60)
                conn = None
                retry_count -= 1

        if conn is None:
            logging.error(f"Failed to establish connection with {self.URL}")

        return conn

    def execute_command(self, command):
        try:
            return self.loop.run_until_complete(self.__execute_command(command))
        except Exception as ex:
            logging.warning(f"Failed to execute {command =}.")
        return False

    async def __execute_command(self, command):
        if self.conn is None:
            logging.debug(f"Connection object is None. creating connection with {self.URL}")
            self.conn = await self.get_connection()
        await self.conn.send(command)
        return await self.conn.recv()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='parser')
    parser.add_argument('ip_addr', type=str, help='ip address of websocket server')
    parser.add_argument('port_no', type=str, help='port number client connection to websocket server')
    args = parser.parse_args()	
	
    ip_address = args.ip_addr
    port=args.port_no
    wb = WebsocketConnector(ip_address=ip_address, port=port)
    command = '/opt/fox/bin/tools/setval CMP:BLK.SPT 40.4'                          # set controller set-point to 40.5 on Foxboro IA series CP via Workstation remote execute
    exit_code = wb.execute_command(command)
    if exit_code == '0':
        print(f'Successfully executed command \'{command}\' on machine {ip_address}')
    else:
        print(f'Could not execute command \'{command}\' on machine {ip_address}')