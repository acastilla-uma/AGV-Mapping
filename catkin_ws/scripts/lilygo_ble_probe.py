#!/usr/bin/env python3
"""Probe a LilyGO T-Echo directly over Bluetooth LE from a PC.

This script is intentionally PC-only and phase-1-only: it scans BLE devices,
connects to the selected LilyGO, enumerates GATT services, attempts safe reads,
and listens for notifications. It does not forward anything to the AGV.
"""

import argparse
import asyncio
import json
import sys
from datetime import datetime, timezone
from pathlib import Path


STATUS_OK = "OK"
STATUS_NO_DEVICE = "TRANSPORT_NOT_CONFIRMED"
STATUS_CONNECT_FAILED = "TRANSPORT_NOT_CONFIRMED"
STATUS_NO_PAYLOAD = "NO_GPS_PAYLOAD_OBSERVED"


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def decode_text(data):
    if data is None:
        return ""
    try:
        return bytes(data).decode("utf-8", errors="replace").strip()
    except Exception:
        return ""


def encode_data(data):
    if data is None:
        return {"raw_hex": "", "text": ""}
    raw = bytes(data)
    return {
        "raw_hex": " ".join("{:02x}".format(value) for value in bytearray(raw)),
        "text": decode_text(raw),
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


def print_event(event, **fields):
    bits = [f"[{utc_now()}]", event]
    for key, value in fields.items():
        if value not in (None, "", []):
            bits.append(f"{key}={value}")
    print(" ".join(bits), flush=True)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "PC-only direct Bluetooth LE probe for LilyGO T-Echo. "
            "Scans, connects, enumerates GATT, reads readable characteristics, "
            "and listens for notifications without contacting the AGV."
        )
    )
    parser.add_argument(
        "--name",
        default="LilyGO,T-Echo",
        help="Comma-separated device name substrings to match during scan.",
    )
    parser.add_argument("--address", help="Exact BLE device address/id to connect to.")
    parser.add_argument("--scan-seconds", type=float, default=10.0, help="BLE scan duration.")
    parser.add_argument("--listen-seconds", type=float, default=30.0, help="Notification listen duration.")
    parser.add_argument(
        "--output",
        default="lilygo_ble_probe.jsonl",
        help="JSONL evidence log path on the PC.",
    )
    parser.add_argument("--read", dest="do_read", action="store_true", default=True, help="Read readable GATT characteristics.")
    parser.add_argument("--no-read", dest="do_read", action="store_false", help="Skip characteristic reads.")
    parser.add_argument("--notify", dest="do_notify", action="store_true", default=True, help="Subscribe to notify/indicate characteristics.")
    parser.add_argument("--no-notify", dest="do_notify", action="store_false", help="Skip notifications.")
    parser.add_argument("--dump-services", action="store_true", default=True, help="Print and log discovered services.")
    return parser


def load_bleak():
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print(
            "ERROR: falta la dependencia 'bleak'. En el PC Windows ejecuta:\n"
            "  py -m pip install bleak\n"
            "Luego repite el comando del probe.",
            file=sys.stderr,
        )
        return None, None
    return BleakClient, BleakScanner


def device_name(device):
    return getattr(device, "name", None) or "(sin_nombre)"


def device_address(device):
    return getattr(device, "address", None) or getattr(device, "details", None) or "(sin_direccion)"


async def scan_devices(scanner, args, log):
    print_event("SCAN_START", seconds=args.scan_seconds, name_filter=args.name, address=args.address)
    log.write("scan_start", seconds=args.scan_seconds, name_filter=args.name, address=args.address)
    try:
        devices = await scanner.discover(timeout=args.scan_seconds)
    except Exception as exc:
        print_event("SCAN_ERROR", error=str(exc))
        log.write("scan_error", error=str(exc))
        return None

    candidates = []
    for device in devices:
        name = device_name(device)
        address = device_address(device)
        is_match = False
        if args.address and str(address).lower() == args.address.lower():
            is_match = True
        elif args.name:
            filters = [part.strip().lower() for part in args.name.split(",") if part.strip()]
            is_match = any(part in name.lower() for part in filters)
        elif not args.name and not args.address:
            is_match = True
        if is_match:
            candidates.append(device)
        print_event("DEVICE", name=name, address=address, match=is_match)
        log.write("device", name=name, address=address, match=is_match)

    print_event("SCAN_DONE", devices=len(devices), candidates=len(candidates))
    log.write("scan_done", devices=len(devices), candidates=len(candidates))
    return candidates


def characteristic_properties(char):
    return list(getattr(char, "properties", []) or [])


async def read_characteristic(client, service, char, log):
    try:
        data = await client.read_gatt_char(char.uuid)
        payload = encode_data(data)
        print_event(
            "READ",
            service=service.uuid,
            char=char.uuid,
            text=payload["text"],
            raw_hex=payload["raw_hex"],
        )
        log.write(
            "read",
            service_uuid=service.uuid,
            characteristic_uuid=char.uuid,
            properties=characteristic_properties(char),
            **payload,
        )
        return bool(payload["raw_hex"] or payload["text"])
    except Exception as exc:
        print_event("READ_ERROR", service=service.uuid, char=char.uuid, error=str(exc))
        log.write(
            "read_error",
            service_uuid=service.uuid,
            characteristic_uuid=char.uuid,
            properties=characteristic_properties(char),
            error=str(exc),
        )
        return False


async def subscribe_characteristic(client, service, char, log, payload_seen):
    def callback(_, data):
        payload = encode_data(data)
        payload_seen["value"] = True
        print_event(
            "NOTIFY",
            service=service.uuid,
            char=char.uuid,
            text=payload["text"],
            raw_hex=payload["raw_hex"],
        )
        log.write(
            "notify",
            service_uuid=service.uuid,
            characteristic_uuid=char.uuid,
            properties=characteristic_properties(char),
            **payload,
        )

    try:
        await client.start_notify(char.uuid, callback)
        print_event("NOTIFY_START", service=service.uuid, char=char.uuid)
        log.write(
            "notify_start",
            service_uuid=service.uuid,
            characteristic_uuid=char.uuid,
            properties=characteristic_properties(char),
        )
        return True
    except Exception as exc:
        print_event("NOTIFY_ERROR", service=service.uuid, char=char.uuid, error=str(exc))
        log.write(
            "notify_error",
            service_uuid=service.uuid,
            characteristic_uuid=char.uuid,
            properties=characteristic_properties(char),
            error=str(exc),
        )
        return False


async def probe_device(client_class, device, args, log):
    name = device_name(device)
    address = device_address(device)
    print_event("CONNECT_START", name=name, address=address)
    log.write("connect_start", name=name, address=address)

    try:
        async with client_class(device) as client:
            if not client.is_connected:
                print_event("CONNECT_FAILED", name=name, address=address)
                log.write("connect_failed", name=name, address=address)
                return STATUS_CONNECT_FAILED

            print_event("CONNECTED", name=name, address=address)
            log.write("connected", name=name, address=address)

            if hasattr(client, "get_services"):
                services = await client.get_services()
            else:
                services = client.services
            service_count = len(list(services))
            payload_seen = {"value": False}
            notify_started = []

            for service in services:
                print_event("SERVICE", uuid=service.uuid, description=getattr(service, "description", ""))
                log.write("service", service_uuid=service.uuid, description=getattr(service, "description", ""))
                for char in service.characteristics:
                    props = characteristic_properties(char)
                    print_event("CHAR", service=service.uuid, char=char.uuid, props=",".join(props))
                    log.write(
                        "characteristic",
                        service_uuid=service.uuid,
                        characteristic_uuid=char.uuid,
                        properties=props,
                        description=getattr(char, "description", ""),
                    )
                    if args.do_read and "read" in props:
                        if await read_characteristic(client, service, char, log):
                            payload_seen["value"] = True
                    if args.do_notify and ("notify" in props or "indicate" in props):
                        if await subscribe_characteristic(client, service, char, log, payload_seen):
                            notify_started.append(char.uuid)

            if service_count == 0:
                print_event("TRANSPORT_NOT_CONFIRMED", reason="no_gatt_services")
                log.write("status", status=STATUS_NO_DEVICE, reason="no_gatt_services")
                return STATUS_NO_DEVICE

            if notify_started:
                print_event("LISTEN_START", seconds=args.listen_seconds, subscriptions=len(notify_started))
                log.write("listen_start", seconds=args.listen_seconds, subscriptions=len(notify_started))
                await asyncio.sleep(args.listen_seconds)
                for char_uuid in notify_started:
                    try:
                        await client.stop_notify(char_uuid)
                    except Exception:
                        pass
                print_event("LISTEN_DONE", payload_seen=payload_seen["value"])
                log.write("listen_done", payload_seen=payload_seen["value"])

            if not payload_seen["value"]:
                print_event("NO_GPS_PAYLOAD_OBSERVED", reason="no_read_or_notify_payload")
                log.write("status", status=STATUS_NO_PAYLOAD, reason="no_read_or_notify_payload")
                return STATUS_NO_PAYLOAD

            print_event("OK", evidence=args.output)
            log.write("status", status=STATUS_OK, output=args.output)
            return STATUS_OK
    except Exception as exc:
        print_event("CONNECT_ERROR", name=name, address=address, error=str(exc))
        log.write("connect_error", name=name, address=address, error=str(exc))
        print_event("TRANSPORT_NOT_CONFIRMED", reason="connect_error")
        log.write("status", status=STATUS_CONNECT_FAILED, reason="connect_error")
        return STATUS_CONNECT_FAILED


async def async_main(args):
    client_class, scanner = load_bleak()
    if client_class is None:
        return 2

    log = JsonlLogger(args.output)
    try:
        log.write("probe_start", argv=sys.argv[1:])
        candidates = await scan_devices(scanner, args, log)
        if candidates is None:
            print_event("TRANSPORT_NOT_CONFIRMED", reason="scan_error", output=args.output)
            log.write("status", status=STATUS_NO_DEVICE, reason="scan_error", output=args.output)
            return 3
        if not candidates:
            print_event("TRANSPORT_NOT_CONFIRMED", reason="no_matching_device", output=args.output)
            log.write("status", status=STATUS_NO_DEVICE, reason="no_matching_device", output=args.output)
            return 3
        if args.address:
            selected = candidates[0]
        elif len(candidates) == 1:
            selected = candidates[0]
        else:
            print("\nVarios candidatos encontrados. Repite con --address <direccion> usando una de estas opciones:")
            for device in candidates:
                print(f"  --address {device_address(device)}    # {device_name(device)}")
            log.write("status", status="MULTIPLE_CANDIDATES", candidates=len(candidates), output=args.output)
            return 4

        status = await probe_device(client_class, selected, args, log)
        return 0 if status == STATUS_OK else 5
    finally:
        log.write("probe_end")
        log.close()
        print_event("LOG_SAVED", output=args.output)


def main():
    args = build_parser().parse_args()
    try:
        if hasattr(asyncio, "run"):
            return asyncio.run(async_main(args))
        loop = asyncio.get_event_loop()
        return loop.run_until_complete(async_main(args))
    except KeyboardInterrupt:
        print_event("INTERRUPTED")
        return 130


if __name__ == "__main__":
    sys.exit(main())
