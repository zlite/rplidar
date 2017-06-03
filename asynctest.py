import time
import asyncio

start = time.time()

def tic():
    return 'at %1.1f seconds' % (time.time() - start)

async def gr1():
    # Busy waits for a tenth of a second, but we don't want to stick around...
    print('10Hz loop started work: {}'.format(tic()))
    time1 = time.time()
    while True:
      # do some work here
      await asyncio.sleep(0)
      if time.time() > time1 + 0.1:
        print('10Hz loop ended work: {}'.format(tic()))
        time1 = time.time()

async def gr2():
    # Busy waits for a second, but we don't want to stick around...
    print('1 Hz loop started work: {}'.format(tic()))
    time2 = time.time()
    while True:
      # do some work here
      await asyncio.sleep(0)
      if time.time() > time2 + 2:
        print('1 Hz loop ended work: {}'.format(tic()))
        time2 = time.time()

async def gr3():
    print("Let's do some stuff while the coroutines are blocked, {}".format(tic()))
    time3 = time.time()
    while True:
      # do some work here
      if time.time() > time3 + 2:
        print("Done!")
      await asyncio.sleep(0)

ioloop = asyncio.get_event_loop()
tasks = [
    ioloop.create_task(gr1()),
    ioloop.create_task(gr2()),
    ioloop.create_task(gr3())
]
ioloop.run_until_complete(asyncio.wait(tasks))
ioloop.close()
