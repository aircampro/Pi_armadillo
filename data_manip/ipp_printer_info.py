#!/usr/bin/python
#
#  pip3 install pyipp
#  Example of using IPP (Internet Printing Protocol) to find the ink left on each cartridgg
#
import asyncio
from pyipp import IPP, Printer
import sys

if len(sys.argv) >= 1:
    PRINTER_IP_ADDR=str(sys.argv[1])
else:
    PRINTER_IP_ADDR="10.0.9.3"

async def main():
    """Show example of connecting to your IPP print server."""
    async with IPP(f"ipps://[{PRINTER_IP_ADDR}]:631/ipp/print") as ipp:
        printer: Printer = await ipp.printer()
        print(printer)
		p_lines=printer.split("\n")
        for pp in p_lines:
            if not (pp.split(",")[0].split("=")[1].find("marker") == -1):
		        print(f"ink color {pp.split(",")[2].split("=")[1]} left until low {d.split(",")[4].split("=")[1] - d.split(",")[5].split("=")[1]}")
            elif not (pp.split(",")[0].split("=")[1].find("manufacturer") == -1):
		        print(f"{pp.split(",")[2].split("=")[1]}")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())