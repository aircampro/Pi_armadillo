# example of sartorius scale
# Minebea Intec Signum
# Sartorius Entris
# Sartorius Miras
# https://github.com/numat/sartorius/tree/master
#
import asyncio
import sartorius

SCALEIP='192.168.1.1'
async def get():
    async with sartorius.Scale(SCALEIP) as scale:
        await scale.zero()             # Zero and tare the scale
        print(await scale.get())       # Get mass, units, stability
        print(await scale.get_info())  # Get model, serial, software version

asyncio.run(get())