#!/usr/bin/env python3
"""Probe a LilyGO T-Echo over Bluetooth LE without contacting the AGV.

This is an operator-side diagnostic script. It scans for BLE devices, connects
to the selected LilyGO, enumerates GATT services, tries safe reads, subscribes
to notify/indicate characteristics, and writes JSONL evidence.
"""

import argparse
import asyncio
import json
import sys
from datetime import datetime, timezone
from pathlib import Path


STATUS_GPS_PAYLOAD = "GPS_PAYLOAD_OBSERVED"
STATUS_TRANSPORT_PAYLOAD = "TRANSPORT_PAYLOAD_OBSERVED"
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


def parse_key_value_status(text):
    parsed = {}
    for token in str(text or "").replace("\r", " ").replace("\n", " ").split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parsed[key.strip().lower()] = value.strip()
    return parsed


def looks_like_gps_payload(payload):
    text = payload.get("text", "")
    upper = text.upper()
    if upper.startswith("$") and ("GGA" in upper[:8] or "RMC" in upper[:8]):
        return True
    parsed = parse_key_value_status(text)
    has_lat = any(key in parsed for key in ("lat", "latitude"))
    has_lon = any(key in parsed for key in ("lon", "lng", "longitude"))
    has_quality = any(key in parsed for key in ("sats", "satellites", "hdop", "fix", "fix_ok", "age_ms", "measurement_age_ms"))
    return has_lat and has_lon and has_quality


def mark_payload(payload_seen, payload):
    payload_seen["any"] = True
    if looks_like_gps_payload(payload):
        payload_seen["gps"] = True


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


def print_event(event, **fields):
    bits = ["[{}]".format(utc_now()), event]
    for key, value in fields.items():
        if value not in (None, "", []):
            bits.append("{}={}".format(key, value))
    print(" ".join(bits), flush=True)


def load_bleak():
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print(
            "ERROR: missing dependency 'bleak'. On the operator computer run:\n"
            "  python -m pip install bleak\n"
            "Then repeat the probe command.",
            file=sys.stderr,
        )
        return None, None
    return BleakClient, BleakScanner


def device_name(device):
    return getattr(device, "name", None) or "(unnamed)"


def device_address(device):
    return getattr(device, "address", None) or getattr(device, "details", None) or "(no_address)"


def characteristic_properties(char):
    return list(getattr(char, "properties", []) or [])


def name_filters(value):
    return [part.strip().lower() for part in value.split(",") if part.strip()]


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
    filters = name_filters(args.name)
    for device in devices:
        name = device_name(device)
        address = str(device_address(device))
        if args.address:
            is_match = address.lower() == args.address.lower()
        elif filters:
            is_match = any(part in name.lower() for part in filters)
        else:
            is_match = True
        if is_match:
            candidates.append(device)
        print_event("DEVICE", name=name, address=address, match=is_match)
        log.write("device", name=name, address=address, match=is_match)

    print_event("SCAN_DONE", devices=len(devices), candidates=len(candidates))
    log.write("scan_done", devices=len(devices), candidates=len(candidates))
    return candidates


async def read_characteristic(client, service, char, log, payload_seen):
    try:
        data = await client.read_gatt_char(char.uuid)
        payload = encode_data(data)
        print_event("READ", service=service.uuid, char=char.uuid, text=payload["text"], raw_hex=payload["raw_hex"])
        log.write(
            "read",
            service_uuid=service.uuid,
            characteristic_uuid=char.uuid,
            properties=characteristic_properties(char),
            **payload
        )
        mark_payload(payload_seen, payload)
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
        mark_payload(payload_seen, payload)
        print_event("NOTIFY", service=service.uuid, char=char.uuid, text=payload["text"], raw_hex=payload["raw_hex"])
        log.write(
            "notify",
            service_uuid=service.uuid,
            characteristic_uuid=char.uuid,
            properties=characteristic_properties(char),
            **payload
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

            service_list = list(services)
            payload_seen = {"any": False, "gps": False}
            notify_started = []
            for service in service_list:
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
                        await read_characteristic(client, service, char, log, payload_seen)
                    if args.do_notify and ("notify" in props or "indicate" in props):
                        if await subscribe_characteristic(client, service, char, log, payload_seen):
                            notify_started.append(char.uuid)

            if not service_list:
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
                print_event("LISTEN_DONE", payload_seen=payload_seen["any"], gps_payload_seen=payload_seen["gps"])
                log.write("listen_done", payload_seen=payload_seen["any"], gps_payload_seen=payload_seen["gps"])

            if not payload_seen["any"]:
                print_event("NO_GPS_PAYLOAD_OBSERVED", reason="no_read_or_notify_payload")
                log.write("status", status=STATUS_NO_PAYLOAD, reason="no_read_or_notify_payload")
                return STATUS_NO_PAYLOAD
            if payload_seen["gps"]:
                print_event("GPS_PAYLOAD_OBSERVED", evidence=args.output)
                log.write("status", status=STATUS_GPS_PAYLOAD, output=args.output)
                return STATUS_GPS_PAYLOAD
            print_event("TRANSPORT_PAYLOAD_OBSERVED", reason="payload_not_gps_shaped", evidence=args.output)
            log.write("status", status=STATUS_TRANSPORT_PAYLOAD, reason="payload_not_gps_shaped", output=args.output)
            if args.require_gps_payload:
                return STATUS_NO_PAYLOAD

            return STATUS_TRANSPORT_PAYLOAD
    except Exception as exc:
        print_event("CONNECT_ERROR", name=name, address=address, error=str(exc))
        log.write("connect_error", name=name, address=address, error=str(exc))
        print_event("TRANSPORT_NOT_CONFIRMED", reason="connect_error")
        log.write("status", status=STATUS_CONNECT_FAILED, reason="connect_error")
        return STATUS_CONNECT_FAILED


def build_parser():
    parser = argparse.ArgumentParser(description="Direct BLE probe for LilyGO T-Echo GPS payloads.")
    parser.add_argument("--name", default="LilyGO,T-Echo", help="Comma-separated BLE name substrings.")
    parser.add_argument("--address", help="Exact BLE address/id to connect to.")
    parser.add_argument("--scan-seconds", type=float, default=10.0)
    parser.add_argument("--listen-seconds", type=float, default=30.0)
    parser.add_argument("--output", default="lilygo_ble_probe.jsonl", help="JSONL evidence path.")
    parser.add_argument("--read", dest="do_read", action="store_true", default=True)
    parser.add_argument("--no-read", dest="do_read", action="store_false")
    parser.add_argument("--notify", dest="do_notify", action="store_true", default=True)
    parser.add_argument("--no-notify", dest="do_notify", action="store_false")
    parser.add_argument("--require-gps-payload", action="store_true", help="Return failure unless payloads look like GPS data.")
    return parser


async def async_main(args):
    client_class, scanner = load_bleak()
    if client_class is None:
        return 2

    log = JsonlLogger(args.output)
    try:
        log.write("probe_start", argv=sys.argv[1:])
        candidates = await scan_devices(scanner, args, log)
        if candidates is None:
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
            print("")
            print("Multiple candidates found. Repeat with --address using one of:")
            for device in candidates:
                print("  --address {}    # {}".format(device_address(device), device_name(device)))
            log.write("status", status="MULTIPLE_CANDIDATES", candidates=len(candidates), output=args.output)
            return 4

        status = await probe_device(client_class, selected, args, log)
        return 0 if status in (STATUS_GPS_PAYLOAD, STATUS_TRANSPORT_PAYLOAD) else 5
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
