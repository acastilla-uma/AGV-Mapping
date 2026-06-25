#!/usr/bin/env python3
"""Forward LilyGO T-Echo BLE notifications from the PC to the AGV/Xavier TCP server."""

import argparse
import asyncio
import json
import socket
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def decode_text(data):
    try:
        return bytes(data).decode("utf-8", errors="replace").strip()
    except Exception:
        return ""


def parse_key_value_status(text):
    parsed = {}
    for token in text.replace("\r", " ").replace("\n", " ").split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parsed[key.strip()] = value.strip()
    return parsed


def parse_payload(data, service_uuid, char_uuid):
    raw = bytes(data)
    text = decode_text(raw)
    parsed = parse_key_value_status(text)
    return {
        "source": "lilygo_ble",
        "time_utc": utc_now(),
        "service_uuid": service_uuid,
        "characteristic_uuid": char_uuid,
        "raw_hex": " ".join("{:02x}".format(value) for value in bytearray(raw)),
        "text": text,
        "parsed": parsed,
    }


class JsonlLogger:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("a", encoding="utf-8")

    def write(self, event, **fields):
        record = {"time_utc": utc_now(), "event": event}
        record.update(fields)
        self.handle.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
        self.handle.flush()

    def close(self):
        self.handle.close()


class TcpForwarder:
    def __init__(self, host, port, timeout, log):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.log = log
        self.sock = None

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def connect(self):
        if self.sock is not None:
            return
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        self.sock = sock
        print("[{}] TCP_CONNECTED host={} port={}".format(utc_now(), self.host, self.port), flush=True)
        self.log.write("tcp_connected", host=self.host, port=self.port)

    def send_json(self, payload):
        line = json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n"
        try:
            self.connect()
            self.sock.sendall(line.encode("utf-8"))
            self.log.write("tcp_sent", bytes=len(line.encode("utf-8")), text=payload.get("text", ""))
            return True
        except Exception as exc:
            print("[{}] TCP_SEND_ERROR error={}".format(utc_now(), exc), flush=True)
            self.log.write("tcp_send_error", error=str(exc), text=payload.get("text", ""))
            self.close()
            return False


def load_bleak():
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print(
            "ERROR: falta 'bleak'. En el PC Windows ejecuta:\n"
            "  py -m pip install bleak",
            file=sys.stderr,
        )
        return None, None
    return BleakClient, BleakScanner


def device_name(device):
    return getattr(device, "name", None) or "(sin_nombre)"


def device_address(device):
    return getattr(device, "address", None) or "(sin_direccion)"


async def select_device(scanner, args, log):
    print("[{}] BLE_SCAN_START seconds={} address={} name={}".format(
        utc_now(), args.scan_seconds, args.address, args.name), flush=True)
    devices = await scanner.discover(timeout=args.scan_seconds)
    candidates = []
    filters = [part.strip().lower() for part in args.name.split(",") if part.strip()]
    for device in devices:
        name = device_name(device)
        address = device_address(device)
        match = False
        if args.address and address.lower() == args.address.lower():
            match = True
        elif not args.address and any(part in name.lower() for part in filters):
            match = True
        if match:
            candidates.append(device)
        print("[{}] BLE_DEVICE name={} address={} match={}".format(
            utc_now(), name, address, match), flush=True)
        log.write("ble_device", name=name, address=address, match=match)
    if not candidates:
        raise RuntimeError("No BLE device matched address/name")
    if args.address or len(candidates) == 1:
        return candidates[0]
    raise RuntimeError("Multiple candidates found; rerun with --address")


def char_properties(char):
    return list(getattr(char, "properties", []) or [])


async def bridge_once(client_class, scanner, args, log):
    device = await select_device(scanner, args, log)
    forwarder = TcpForwarder(args.agv_host, args.agv_port, args.tcp_timeout, log)
    payload_count = {"value": 0}

    async with client_class(device) as client:
        print("[{}] BLE_CONNECTED name={} address={}".format(
            utc_now(), device_name(device), device_address(device)), flush=True)
        log.write("ble_connected", name=device_name(device), address=device_address(device))

        if hasattr(client, "get_services"):
            services = await client.get_services()
        else:
            services = client.services

        notify_chars = []
        for service in services:
            for char in service.characteristics:
                props = char_properties(char)
                if "notify" in props or "indicate" in props:
                    notify_chars.append((service.uuid, char.uuid))
                    log.write("notify_candidate", service_uuid=service.uuid, characteristic_uuid=char.uuid, properties=props)

        preferred = [(svc, char) for svc, char in notify_chars if char.lower() == NUS_TX_UUID]
        selected = preferred or notify_chars
        if not selected:
            raise RuntimeError("No notify/indicate BLE characteristics found")

        def make_callback(service_uuid, char_uuid):
            def callback(_, data):
                payload = parse_payload(data, service_uuid, char_uuid)
                payload_count["value"] += 1
                print("[{}] BLE_NOTIFY text={}".format(utc_now(), payload.get("text", "")), flush=True)
                log.write("ble_notify", **payload)
                forwarder.send_json(payload)
            return callback

        subscribed = []
        for service_uuid, char_uuid in selected:
            try:
                await client.start_notify(char_uuid, make_callback(service_uuid, char_uuid))
                subscribed.append(char_uuid)
                print("[{}] BLE_NOTIFY_START service={} char={}".format(
                    utc_now(), service_uuid, char_uuid), flush=True)
                log.write("ble_notify_start", service_uuid=service_uuid, characteristic_uuid=char_uuid)
            except Exception as exc:
                log.write("ble_notify_error", service_uuid=service_uuid, characteristic_uuid=char_uuid, error=str(exc))

        if not subscribed:
            raise RuntimeError("Could not subscribe to any notify/indicate characteristic")

        print("[{}] BRIDGE_RUNNING seconds={}".format(utc_now(), args.listen_seconds), flush=True)
        log.write("bridge_running", seconds=args.listen_seconds)
        await asyncio.sleep(args.listen_seconds)

        for char_uuid in subscribed:
            try:
                await client.stop_notify(char_uuid)
            except Exception:
                pass
        forwarder.close()

    if payload_count["value"] == 0:
        raise RuntimeError("No BLE payload received during listen window")
    return payload_count["value"]


def build_parser():
    parser = argparse.ArgumentParser(description="Forward LilyGO BLE notifications to the AGV/Xavier TCP metadata logger.")
    parser.add_argument("--address", help="BLE address of the LilyGO, e.g. CE:BA:33:E1:3A:39.")
    parser.add_argument("--name", default="LilyGO,T-Echo", help="Comma-separated BLE name filters when --address is omitted.")
    parser.add_argument("--scan-seconds", type=float, default=10.0)
    parser.add_argument("--listen-seconds", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--agv-host", default="100.123.78.14", help="Xavier/AGV Tailscale or LAN IP.")
    parser.add_argument("--agv-port", type=int, default=29500)
    parser.add_argument("--tcp-timeout", type=float, default=5.0)
    parser.add_argument("--output", default="lilygo_tcp_bridge.jsonl")
    return parser


async def async_main(args):
    client_class, scanner = load_bleak()
    if client_class is None:
        return 2
    log = JsonlLogger(args.output)
    try:
        log.write("bridge_start", argv=sys.argv[1:], agv_host=args.agv_host, agv_port=args.agv_port)
        if args.listen_seconds <= 0:
            args.listen_seconds = 365 * 24 * 3600
        count = await bridge_once(client_class, scanner, args, log)
        log.write("bridge_done", payload_count=count)
        print("[{}] BRIDGE_DONE payload_count={} output={}".format(utc_now(), count, args.output), flush=True)
        return 0
    except KeyboardInterrupt:
        log.write("interrupted")
        print("[{}] INTERRUPTED".format(utc_now()), flush=True)
        return 130
    except Exception as exc:
        log.write("bridge_error", error=str(exc))
        print("[{}] BRIDGE_ERROR error={}".format(utc_now(), exc), flush=True)
        return 1
    finally:
        log.write("bridge_end")
        log.close()


def main():
    args = build_parser().parse_args()
    if hasattr(asyncio, "run"):
        return asyncio.run(async_main(args))
    loop = asyncio.get_event_loop()
    return loop.run_until_complete(async_main(args))


if __name__ == "__main__":
    sys.exit(main())
