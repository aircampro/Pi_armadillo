#! /usr/bin/env python3
#
# OCPP client for EV charging points for example of simple servers refer to main API site https://github.com/mobilityhouse/ocpp/tree/master/examples/v201
#
# the example with v16 is for use with steve server 
# git clone --depth 1 https://github.com/RWTH-i5-IDSG/steve
# cd steve
# docker-compose up -d
#
import asyncio
import logging
import websockets
import concurrent.futures
logging.basicConfig(level=logging.INFO)

###########################
# Parameters
###########################
CP_ID = "my-cp-001"              # EV charger
CP_VENDOR = "MY-VENDOR"          # "The Mobility House" v16 'anewone' v201
CP_SERIAL = "MY-SERIAL-001"      # charge point serial number sent with V16
CP_MODEL = "MY-MODEL"            # "Optimus" v16 or 'Wallbox XYZ' v201
CP_REASON = "PowerUp"            # reason is sent with V201

# ===================== v16 ============================
from ocpp.v16 import call, call_result
from ocpp.routing import on
from ocpp.v16 import ChargePoint as cp
from ocpp.v16.call_result import (
 BootNotificationPayload,
)
from ocpp.v16.enums import (
 DataTransferStatus,
 Action,
)

# charger point class v16
class ChargePoint16(cp):
    @on(Action.DataTransfer)
    async def respond_datatransfer(self, vendor_id, message_id, data):
    print(f"DataTransfer Vendor ID -> {vendor_id}")
    print(f"DataTransfer Message ID -> {message_id}")
    print(f"Datatransfer Data -> {data}")

    if vendor_id != CP_VENDOR:
        message = f"{CP_ID}:NG Vendor ID ({vendor_id}) not valid , please set correct vendor id"
        return call_result.DataTransferPayload(DataTransferStatus.rejected, message)
    else:
        message = f"{CP_ID}:OK"
        return call_result.DataTransferPayload(DataTransferStatus.accepted, message)

    async def send_boot_notification(self, model, serial, vendor):
        req = call.BootNotificationPayload( charge_point_model=model, charge_point_serial_number=serial, charge_point_vendor=vendor, )
        response: BootNotificationPayload = await self.call(req)
        print(f"Res -> {response}")
        return response

# ==================== v201 ============================
from ocpp.v201.enums import RegistrationStatusType
from ocpp.v201 import call
from ocpp.v201 import ChargePoint as cp

class ChargePointV201(cp):
    @on(Action.DataTransfer)
    async def send_boot_notification(self, model, reasn, vendor):
        request = call.BootNotificationPayload(
            charging_station={
                'model': model,
                'vendor_name': vendor
            },
            reason=reasn
        )
        response = await self.call(request)

        if response.status == RegistrationStatusType.accepted:
            print("Connected to central system.")
			
# main running thread v16
async def main1():
    WS_ENDPOINT = f"ws://localhost:8180/steve/websocket/CentralSystemService/{CP_ID}"
    async with websockets.connect(WS_ENDPOINT, subprotocols=["ocpp1.6"]) as ws:
    cp = ChargePoint16(CP_ID, ws, response_timeout=5)
    await asyncio.gather( cp.start(), cp.send_boot_notification(CP_MODEL, CP_SERIAL, CP_VENDOR), )

# true concurrent thread v16
async def main2():
    WS_ENDPOINT = f"ws://localhost:8180/steve/websocket/CentralSystemService/{CP_ID}"
    async with websockets.connect(WS_ENDPOINT, subprotocols=["ocpp1.6"]) as ws:
    cp = ChargePoint16(CP_ID, ws, response_timeout=5)
    def start():
        cp.start()
    def send_boot():
        cp.send_boot_notification(CP_MODEL, CP_SERIAL, CP_VENDOR)
    loop = asyncio.get_running_loop()
    with concurrent.futures.ProcessPoolExecutor() as pool:
        task1 = loop.run_in_executor(pool, start)
        task2 = loop.run_in_executor(pool, send_boot)
        result1 = await task1
        result2 = await task2
        print('result:', result1, result2)

# main running thread v201
async def main3():
    WS_ENDPOINT = f"ws://localhost:9000/{CP_ID}"
    async with websockets.connect(WS_ENDPOINT, subprotocols=["ocpp2.0.1"]) as ws:
    cp = ChargePointV201(CP_ID, ws, response_timeout=5)
    await asyncio.gather( cp.start(), cp.send_boot_notification(CP_MODEL, CP_REASON, CP_VENDOR), )

# true concurrent thread v201
async def main4():
    WS_ENDPOINT = f"ws://localhost:9000/{CP_ID}"
    async with websockets.connect(WS_ENDPOINT, subprotocols=["ocpp2.0.1"]) as ws:
    cp = ChargePointV201(CP_ID, ws, response_timeout=5)
    def start():
        cp.start()
    def send_boot():
        cp.send_boot_notification(CP_MODEL, CP_REASON, CP_VENDOR)
    loop = asyncio.get_running_loop()
    with concurrent.futures.ProcessPoolExecutor() as pool:
        task1 = loop.run_in_executor(pool, start)
        task2 = loop.run_in_executor(pool, send_boot)
        result1 = await task1
        result2 = await task2
        print('result:', result1, result2)
		
#choose asyncio thread method and protocol v16 or v201 (1-4)
THM=1
 
if __name__ == "__main__":

    if THM == 1 :
        try:
            # asyncio.run() is used when running this example with Python 3.7 and higher.
            asyncio.run(main1())
        except AttributeError:
            # For Python 3.6 a bit more code is required to run the main() task on
            # an event loop.
            loop = asyncio.get_event_loop()
            loop.run_until_complete(main1())
            loop.close()
    elif THM == 2 :
        try:
            # asyncio.run() is used when running this example with Python 3.7 and higher.
            asyncio.run(main2())
        except AttributeError:
            # For Python 3.6 a bit more code is required to run the main() task on
            # an event loop.
            loop = asyncio.get_event_loop()
            loop.run_until_complete(main2())
            loop.close()
    elif THM == 3 :
        try:
            # asyncio.run() is used when running this example with Python 3.7 and higher.
            asyncio.run(main3())
        except AttributeError:
            # For Python 3.6 a bit more code is required to run the main() task on
            # an event loop.
            loop = asyncio.get_event_loop()
            loop.run_until_complete(main3())
            loop.close()
    elif THM == 4 :
        try:
            # asyncio.run() is used when running this example with Python 3.7 and higher.
            asyncio.run(main4())
        except AttributeError:
            # For Python 3.6 a bit more code is required to run the main() task on
            # an event loop.
            loop = asyncio.get_event_loop()
            loop.run_until_complete(main4())
            loop.close()