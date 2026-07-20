#!/usr/bin/env python3
"""Forward LilyGO T-Echo BLE notifications to a TCP GPS metadata receiver."""

import argparse
import asyncio
import json
import socket
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def decode_text(data):
    try:
        return bytes(data).decode("utf-8", errors="replace").strip()
    except Exception:
        return ""


def raw_hex(data):
    return " ".join("{:02x}".format(value) for value in bytearray(bytes(data)))


def parse_key_value_status(text):
    parsed = {}
    for token in text.replace("\r", " ").replace("\n", " ").split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parsed[key.strip()] = value.strip()
    return parsed


def parse_payload(data, service_uuid, char_uuid):
    text = decode_text(data)
    return {
        "source": "lilygo_ble",
        "time_utc": utc_now(),
        "service_uuid": service_uuid,
        "characteristic_uuid": char_uuid,
        "raw_hex": raw_hex(data),
        "text": text,
        "parsed": parse_key_value_status(text),
    }


class JsonlLogger:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("a", encoding="utf-8")

    def write(self, event, **fields):
        record = {"time_utc": utc_now(), "event": event}
        record.update(fields)
        self.handle.write(json.dumps(record, ensure_ascii=True, sort_keys=True) + "\n")
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
        line = json.dumps(payload, ensure_ascii=True, sort_keys=True) + "\n"
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


async def tcp_sender(queue, forwarder, log):
    loop = asyncio.get_event_loop()
    while True:
        payload = await queue.get()
        try:
            if payload is None:
                return
            await loop.run_in_executor(None, forwarder.send_json, payload)
        finally:
            queue.task_done()


async def stop_tcp_sender(queue, sender_task, forwarder):
    try:
        if sender_task.done():
            sender_task.result()
            return
        await queue.join()
        await queue.put(None)
        await sender_task
    finally:
        forwarder.close()


def load_bleak():
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print(
            "ERROR: missing dependency 'bleak'. On the operator computer run:\n"
            "  python -m pip install bleak",
            file=sys.stderr,
        )
        return None, None
    return BleakClient, BleakScanner


def device_name(device):
    return getattr(device, "name", None) or "(unnamed)"


def device_address(device):
    return getattr(device, "address", None) or "(no_address)"


def char_properties(char):
    return list(getattr(char, "properties", []) or [])


async def select_device(scanner, args, log):
    print("[{}] BLE_SCAN_START seconds={} address={} name={}".format(
        utc_now(), args.scan_seconds, args.address, args.name), flush=True)
    devices = await scanner.discover(timeout=args.scan_seconds)
    candidates = []
    filters = [part.strip().lower() for part in args.name.split(",") if part.strip()]
    for device in devices:
        name = device_name(device)
        address = str(device_address(device))
        if args.address:
            match = address.lower() == args.address.lower()
        else:
            match = any(part in name.lower() for part in filters)
        if match:
            candidates.append(device)
        print("[{}] BLE_DEVICE name={} address={} match={}".format(utc_now(), name, address, match), flush=True)
        log.write("ble_device", name=name, address=address, match=match)
    if not candidates:
        raise RuntimeError("No BLE device matched address/name")
    if args.address or len(candidates) == 1:
        return candidates[0]
    raise RuntimeError("Multiple candidates found; rerun with --address")


async def bridge_once(client_class, scanner, args, log):
    device = await select_device(scanner, args, log)
    forwarder = TcpForwarder(args.jetson_host, args.jetson_port, args.tcp_timeout, log)
    send_queue = asyncio.Queue(maxsize=args.tcp_queue_size)
    sender_task = asyncio.ensure_future(tcp_sender(send_queue, forwarder, log))
    payload_count = {"value": 0}

    try:
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
                    try:
                        send_queue.put_nowait(payload)
                    except asyncio.QueueFull:
                        log.write("tcp_queue_full", text=payload.get("text", ""))
                        print("[{}] TCP_QUEUE_FULL dropped_payload=1".format(utc_now()), flush=True)
                return callback

            subscribed = []
            for service_uuid, char_uuid in selected:
                try:
                    await client.start_notify(char_uuid, make_callback(service_uuid, char_uuid))
                    subscribed.append(char_uuid)
                    print("[{}] BLE_NOTIFY_START service={} char={}".format(utc_now(), service_uuid, char_uuid), flush=True)
                    log.write("ble_notify_start", service_uuid=service_uuid, characteristic_uuid=char_uuid)
                except Exception as exc:
                    log.write("ble_notify_error", service_uuid=service_uuid, characteristic_uuid=char_uuid, error=str(exc))

            if not subscribed:
                raise RuntimeError("Could not subscribe to any notify/indicate characteristic")

            listen_seconds = args.listen_seconds
            print("[{}] BRIDGE_RUNNING seconds={}".format(utc_now(), listen_seconds), flush=True)
            log.write("bridge_running", seconds=listen_seconds)
            if listen_seconds <= 0:
                while True:
                    await asyncio.sleep(3600)
            else:
                await asyncio.sleep(listen_seconds)

            for char_uuid in subscribed:
                try:
                    await client.stop_notify(char_uuid)
                except Exception:
                    pass
    finally:
        await stop_tcp_sender(send_queue, sender_task, forwarder)

    if args.require_payload and payload_count["value"] == 0:
        raise RuntimeError("No BLE payload received during listen window")
    return payload_count["value"]


def build_parser():
    parser = argparse.ArgumentParser(description="Forward LilyGO BLE notifications to the Jetson GPS metadata TCP receiver.")
    parser.add_argument("--address", help="BLE address/id of the LilyGO.")
    parser.add_argument("--name", default="LilyGO,T-Echo", help="Comma-separated BLE name filters.")
    parser.add_argument("--scan-seconds", type=float, default=10.0)
    parser.add_argument("--listen-seconds", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--jetson-host", default="192.168.8.174", help="Jetson LAN or Tailscale IP.")
    parser.add_argument("--jetson-port", type=int, default=29500)
    parser.add_argument("--tcp-timeout", type=float, default=5.0)
    parser.add_argument("--tcp-queue-size", type=int, default=100)
    parser.add_argument("--reconnect-delay", type=float, default=3.0)
    parser.add_argument("--max-retries", type=int, default=0, help="0 means retry forever.")
    parser.add_argument("--require-payload", action="store_true", help="Fail if no BLE notification arrives before timeout.")
    parser.add_argument("--output", default="lilygo_tcp_bridge.jsonl")
    return parser


async def async_main(args):
    client_class, scanner = load_bleak()
    if client_class is None:
        return 2
    log = JsonlLogger(args.output)
    attempts = 0
    try:
        log.write("bridge_start", argv=sys.argv[1:], jetson_host=args.jetson_host, jetson_port=args.jetson_port)
        while args.max_retries <= 0 or attempts < args.max_retries:
            attempts += 1
            try:
                count = await bridge_once(client_class, scanner, args, log)
                log.write("bridge_done", payload_count=count, attempts=attempts)
                print("[{}] BRIDGE_DONE payload_count={} output={}".format(utc_now(), count, args.output), flush=True)
                return 0
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                log.write("bridge_attempt_error", attempt=attempts, error=str(exc))
                print("[{}] BRIDGE_ATTEMPT_ERROR attempt={} error={}".format(utc_now(), attempts, exc), flush=True)
                if args.max_retries > 0 and attempts >= args.max_retries:
                    break
                time.sleep(args.reconnect_delay)
        log.write("bridge_error", error="retry_limit_reached", attempts=attempts)
        return 1
    except KeyboardInterrupt:
        log.write("interrupted")
        print("[{}] INTERRUPTED".format(utc_now()), flush=True)
        return 130
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
