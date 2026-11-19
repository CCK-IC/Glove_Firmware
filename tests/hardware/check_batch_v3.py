#!/usr/bin/env python3
"""Manual hardware regression; see tests/README.md for configuration and scope."""
import argparse
import asyncio
import collections
import json
import struct
from pathlib import Path
from bleak import BleakClient, BleakScanner
NAME = 'XIAO-IMU-RAW-75F2'
UUID = {1: '8f7c0099-4f54-4d55-8000-5849414f494d', 3: '8f7c0011-4f54-4d55-8000-5849414f494d', 4: '8f7c0021-4f54-4d55-8000-5849414f494d'}
CONTROL = '8f7c0022-4f54-4d55-8000-5849414f494d'
SIZES = {1: 24, 2: 22, 3: 20}

def decode(data, kind):
    b = bytes(data)
    assert b[:3] == bytes([3, kind, 1]), (kind, len(b), b[:4].hex())
    if kind == 4:
        assert len(b) == 108 and b[3] == 1 and (not any(b[56:]))
        o = 4
        return dict(uptime=struct.unpack_from('<I', b, o)[0], rate=struct.unpack_from('<H', b, o + 4)[0], sensor_rate=struct.unpack_from('<H', b, o + 6)[0], packet_rate=struct.unpack_from('<H', b, o + 8)[0], mtu=struct.unpack_from('<H', b, o + 12)[0], firmware=list(b[o + 18:o + 21]), drops=struct.unpack_from('<I', b, o + 28)[0], fifo=struct.unpack_from('<I', b, o + 32)[0], i2c=struct.unpack_from('<I', b, o + 36)[0], sample_drops=struct.unpack_from('<I', b, o + 40)[0], acquired=struct.unpack_from('<I', b, o + 44)[0])
    count = b[3]
    size = SIZES[kind]
    assert 1 <= count <= 8 and len(b) == 28 + count * 2 * size and (not any(b[16:28]))
    seq, ts, rate = struct.unpack_from('<IIH', b, 4)
    out = []
    last = -1
    last_t = 0
    for i in range(count):
        o = 28 + i * 2 * size
        sd, td, flags = struct.unpack_from('<BHB', b, o)
        assert sd == td == 0 if i == 0 else sd > last and td >= last_t
        assert not any(b[o + size:o + 2 * size])
        last = sd
        last_t = td
        row = {'seq': seq + sd & 4294967295, 'ts': ts + td & 4294967295, 'rate': rate, 'flags': flags}
        if kind == 1:
            row['mag'] = struct.unpack_from('<hhh', b, o + 16)
            assert flags & 8 and (not flags & 32), row
        if kind == 3:
            row['values'] = struct.unpack_from('<8h', b, o + 4)
            assert not any(row['values'][b[14]:]), row
        out.append(row)
    return out

class Capture:

    def __init__(self):
        self.rows = {1: [], 3: []}
        self.frames = collections.Counter()
        self.notifications = collections.Counter()
        self.counts = {1: collections.Counter(), 3: collections.Counter()}
        self.errors = []
        self.partial = {}
        self.status = []

    def callback(self, kind):

        def handle(ch, data):
            self.notifications[kind] += 1
            try:
                b = bytes(data)
                if b[0] == 243:
                    marker, k, fid, idx, count, length = struct.unpack_from('<BBHBBH', b)
                    assert k == kind and count > 0 and (length <= 512)
                    if idx == 0:
                        self.partial[k] = (fid, count, length, [])
                    old = self.partial[k]
                    assert old[:3] == (fid, count, length) and idx == len(old[3])
                    old[3].append(b[8:])
                    if idx + 1 < count:
                        return
                    b = b''.join(old[3])
                    del self.partial[k]
                    assert len(b) == length
                result = decode(b, kind)
                self.frames[kind] += 1
                if kind == 4:
                    self.status.append(result)
                else:
                    self.rows[kind].extend(result)
                    self.counts[kind][len(result)] += 1
            except Exception as e:
                self.errors.append(repr(e))
        return handle

    def summary(self):
        gaps = {}
        for k, rows in self.rows.items():
            gaps[k] = sum((max(0, (b['seq'] - a['seq'] & 4294967295) - 1) for a, b in zip(rows, rows[1:])))
        return {'notifications': dict(self.notifications), 'frames': dict(self.frames), 'samples': {k: len(v) for k, v in self.rows.items()}, 'batch_sizes': {k: dict(v) for k, v in self.counts.items()}, 'sequence_gaps': gaps, 'errors': self.errors, 'statuses': self.status[-2:]}

async def main():
    parser = argparse.ArgumentParser(description='Live v3 raw/AUX/status batch regression. Requires Bleak and an available raw-mode sensor. Sends rate/status commands; restores rate45/status1000 before disconnect, after which firmware restores configured defaults.')
    parser.add_argument('--name', default=NAME, help='Exact unique advertising name')
    parser.add_argument('--phase-seconds', type=float, default=15, help='Seconds per 45/90/200/45 Hz phase (at least 2)')
    parser.add_argument('--output', type=Path, default=Path('batch_v3_live_result.json'))
    args = parser.parse_args()
    if args.phase_seconds < 2:
        parser.error('--phase-seconds must be at least 2')
    device = await BleakScanner.find_device_by_name(args.name, timeout=15)
    assert device, args.name + ' not advertising'
    report = {}
    first = Capture()
    async with BleakClient(device, timeout=20) as c:
        before = decode(await c.read_gatt_char(UUID[4]), 4)
        assert before['firmware'] == [1, 4, 0], before
        print('connected ' + json.dumps(before), flush=True)
        for k in (1, 3):
            assert len(decode(await c.read_gatt_char(UUID[k]), k)) == 1
        battery = await c.read_gatt_char('00002a19-0000-1000-8000-00805f9b34fb')
        assert len(battery) == 1
        await c.write_gatt_char(CONTROL, struct.pack('<BH', 2, 250), response=True)
        await c.write_gatt_char(CONTROL, struct.pack('<BH', 1, 45), response=True)
        for k in (1, 3, 4):
            await c.start_notify(UUID[k], first.callback(k))
        for hz, response in [(45, True), (90, True), (200, True), (45, True)]:
            await c.write_gatt_char(CONTROL, struct.pack('<BH', 1, hz), response=response)
            await asyncio.sleep(args.phase_seconds)
            state = decode(await c.read_gatt_char(UUID[4]), 4)
            assert state['rate'] == hz, (hz, state)
            assert c.is_connected
            print('phase ' + json.dumps({'requested': hz, 'response': response, 'state': state, 'samples': {k: len(v) for k, v in first.rows.items()}}), flush=True)
        after = decode(await c.read_gatt_char(UUID[4]), 4)
        for k in (1, 3, 4):
            await c.stop_notify(UUID[k])
        report['continuous'] = first.summary()
        report['before'] = before
        report['after'] = after
        for key in ('drops', 'sample_drops', 'fifo', 'i2c'):
            assert after[key] == before[key], (key, before, after)
        assert not first.errors, first.errors
        assert all((v == 0 for v in report['continuous']['sequence_gaps'].values())), report['continuous']
        assert all((max(first.counts[k]) > 1 for k in (1, 3)))
        await asyncio.sleep(0.3)
        floor = decode(await c.read_gatt_char(UUID[4]), 4)['uptime']
        second = Capture()
        for k in (1, 3, 4):
            await c.start_notify(UUID[k], second.callback(k))
        await asyncio.sleep(4)
        for k in (1, 3, 4):
            await c.stop_notify(UUID[k])
        assert all((rows and rows[0]['ts'] >= floor for rows in second.rows.values())), (floor, second.summary())
        assert not second.errors
        report['resubscribe'] = second.summary()
    await asyncio.sleep(1)
    third = Capture()
    async with BleakClient(device, timeout=20) as c:
        floor = decode(await c.read_gatt_char(UUID[4]), 4)['uptime']
        for k in (1, 3, 4):
            await c.start_notify(UUID[k], third.callback(k))
        await asyncio.sleep(5)
        for k in (1, 3, 4):
            await c.stop_notify(UUID[k])
        assert all((rows and rows[0]['ts'] >= floor for rows in third.rows.values())), (floor, third.summary())
        assert not third.errors
        await c.write_gatt_char(CONTROL, struct.pack('<BH', 2, 1000), response=True)
        report['reconnect'] = third.summary()
    report['harness'] = {'device': args.name, 'requested_rates_hz': [45, 90, 200, 45], 'seconds_per_phase': args.phase_seconds}
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
if __name__ == '__main__':
    asyncio.run(main())
