import asyncio
import time

async def infinite():
    i = 0
    while True:
        i = i + 1  

async def tenhz():  # this is the loop that runs at 10Hz
    now = time.time()
    newtime = 0
    print ("Ten Hz")
    while newtime < (now + 0.1):
        newtime = time.time()
        infinite()
        # do all your 10hz stuff here


async def onehz(): # this is the loop that runs at 1 Hz
    now = time.time()
    newtime = 0
    print ("One Hz")
    while newtime < (now + 1):
        await tenhz()
        newtime = time.time()
        # do all your 1 hz stuff here

loop = asyncio.get_event_loop()  
try:
    while True:    # Run forever
        loop.run_until_complete(onehz())
except KeyboardInterrupt:
    loop.close()  
    
