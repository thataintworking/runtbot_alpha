# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import click
import json
from redis import StrictRedis


COL_HEADINGS = ("gx","gy","gz","ax","ay","az","mx","my","mz","temp")
COL_FORMAT = '{:>7}'


@click.command()
@click.argument('channel', default='motion-sensor-data')
@click.option('--host', '-h', default='localhost')
@click.option('--port', '-p', default=6379)
@click.option('--db', '-d', default=0)
def main(channel, host, port, db):
    r = StrictRedis(host, port, db)
    p = r.pubsub(ignore_subscribe_messages=True)
    p.subscribe(channel)
    print(','.join([COL_FORMAT.format(h) for h in COL_HEADINGS]), flush=True)
    for msg in p.listen():
        # print(msg.get('data', b'').decode('utf-8') + '   ', end='\r')
        data = json.loads(msg.get('data', b'').decode('utf-8'))
        print(','.join([COL_FORMAT.format(data[h]) for h in COL_HEADINGS]), flush=True)

if __name__ == '__main__':
    main()
