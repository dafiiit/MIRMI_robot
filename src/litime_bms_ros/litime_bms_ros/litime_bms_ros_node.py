#!/usr/bin/env python3
"""
ROS2 node that queries a LiTime BMS over BLE and publishes the data.

The BLE protocol is based on:
  https://github.com/calledit/LiTime_BMS_bluetooth
  https://github.com/mirosieber/Litime_BMS_ESP32
"""

import asyncio
import binascii
from typing import List, Optional

from bleak import BleakClient, BleakError

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


DEFAULT_MAC = "C8:47:80:18:AE:45"
DEFAULT_NOTIFY_UUIDS = [
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    "0000ffe3-0000-1000-8000-00805f9b34fb",
]
DEFAULT_WRITE_UUIDS = [
    "0000ffe2-0000-1000-8000-00805f9b34fb",
]

COMMANDS = {
    "product_registration": bytes.fromhex("000004010155aa05"),
    "query_battery_status": bytes.fromhex("000004011355aa17"),
}


def hx(data: bytes) -> str:
    return binascii.hexlify(data).decode()


def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF


def interpret_ack(payload: bytes):
    if len(payload) != 9:
        return None
    if payload[0:2] != b"\x00\x00" or payload[3] != 0x01:
        return None
    if payload[5] != 0x55 or payload[6] != 0xAA:
        return None
    cmd_byte = payload[4]
    if cmd_byte < 0x80:
        return None
    base_cmd = cmd_byte & 0x7F
    status = payload[7]
    reported_chk = payload[8]
    if checksum8(payload[:7] + bytes([status])) != reported_chk:
        return None
    return {"ack_for": base_cmd, "status": status}


def parse_status_frame(payload: bytes):
    if len(payload) < 105:
        return None
    if payload[0:2] != b"\x00\x00" or payload[3] != 0x01:
        return None
    if payload[5] != 0x55 or payload[6] != 0xAA:
        return None

    def u16(offset):
        return int.from_bytes(payload[offset : offset + 2], "little")

    def s16(offset):
        return int.from_bytes(payload[offset : offset + 2], "little", signed=True)

    def u32(offset):
        return int.from_bytes(payload[offset : offset + 4], "little")

    def s32(offset):
        return int.from_bytes(payload[offset : offset + 4], "little", signed=True)

    cell_voltages = []
    for i in range(16, 48, 2):
        value = u16(i)
        if value:
            cell_voltages.append(value / 1000.0)

    return {
        "total_voltage": u32(8) / 1000.0,
        "cells_voltage_sum": u32(12) / 1000.0,
        "cell_voltages": cell_voltages,
        "current": s32(48) / 1000.0,
        "cell_temp": s16(52),
        "mosfet_temp": s16(54),
        "remaining_ah": u16(62) / 100.0,
        "full_capacity_ah": u16(64) / 100.0,
        "soc": u16(90),
        "soh": u32(92),
        "discharge_count": u32(96),
        "discharged_ah": u32(100) / 1000.0,
        "protection_state": payload[76:80][::-1].hex(),
        "heat_state": payload[68:72][::-1].hex(),
        "balance_memory": payload[72:76][::-1].hex(),
        "failure_state": payload[80:83][::-1].hex(),
        "balancing_bits": payload[84:88][::-1].hex(),
        "battery_state": payload[88:90][::-1].hex(),
    }


class LiTimeBMSNode(Node):
    def __init__(self):
        super().__init__("litime_bms_node")

        self.declare_parameter("mac", DEFAULT_MAC)
        self.declare_parameter("query_interval", 1.0)
        self.declare_parameter("write_uuids", DEFAULT_WRITE_UUIDS)
        self.declare_parameter("notify_uuids", DEFAULT_NOTIFY_UUIDS)
        self.declare_parameter("send_registration", True)

        default_query_hex = hx(COMMANDS["query_battery_status"])
        self.declare_parameter("query_command_hex", default_query_hex)

        self._battery_pub = self.create_publisher(BatteryState, "litime_bms/state", 10)
        self._raw_pub = self.create_publisher(String, "litime_bms/raw", 10)

        self._active_notifications: List[str] = []
        self._client: Optional[BleakClient] = None

    async def run(self):
        mac = self.get_parameter("mac").get_parameter_value().string_value
        if not mac:
            self.get_logger().error(
                "Parameter 'mac' ist leer. Setze ihn über ROS2 Parameter oder CLI."
            )
            return

        query_interval = (
            self.get_parameter("query_interval").get_parameter_value().double_value
        )
        write_uuids = list(
            self.get_parameter("write_uuids")
            .get_parameter_value()
            .string_array_value
        )
        if not write_uuids:
            self.get_logger().error("Parameter 'write_uuids' darf nicht leer sein.")
            return

        notify_uuids = list(
            self.get_parameter("notify_uuids")
            .get_parameter_value()
            .string_array_value
        )
        if not notify_uuids:
            self.get_logger().warn(
                "Keine notify_uuids gesetzt – es werden keine Daten empfangen."
            )

        send_registration = (
            self.get_parameter("send_registration")
            .get_parameter_value()
            .bool_value
        )
        query_hex = (
            self.get_parameter("query_command_hex")
            .get_parameter_value()
            .string_value
        )
        try:
            query_payload = bytes.fromhex(query_hex)
        except ValueError:
            self.get_logger().error(
                f"query_command_hex '{query_hex}' ist keine gültige Hex-Zeichenkette."
            )
            return

        registration_payload = COMMANDS["product_registration"]

        reconnect_delay = 5.0

        self.get_logger().info(
            f"Starte LiTime BMS Node für {mac} (Intervall {query_interval}s)."
        )

        while rclpy.ok():
            try:
                await self._run_session(
                    mac,
                    notify_uuids,
                    write_uuids,
                    query_payload,
                    query_interval,
                    registration_payload if send_registration else None,
                )
            except asyncio.CancelledError:
                break
            except BleakError as exc:
                self.get_logger().error(f"BLE-Fehler: {exc}")
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().exception(f"Unerwarteter Fehler: {exc}")

            if rclpy.ok():
                self.get_logger().info(
                    f"Versuche Reconnect in {reconnect_delay:.0f} Sekunden …"
                )
                await asyncio.sleep(reconnect_delay)

        self.get_logger().info("Beende LiTime BMS Node.")

    async def _run_session(
        self,
        mac: str,
        notify_uuids: List[str],
        write_uuids: List[str],
        query_payload: bytes,
        query_interval: float,
        registration_payload: Optional[bytes],
    ):
        # Der Context-Manager VERBINDET bereits in __aenter__().
        async with BleakClient(mac, timeout=20.0) as client:
            self._client = client

            # Kein zweiter connect()-Aufruf hier!
            if not client.is_connected:
                # Sollte praktisch nie passieren – aber zur Sicherheit:
                raise BleakError("Verbindung fehlgeschlagen (nicht verbunden nach __aenter__).")

            self.get_logger().info("BLE-Verbindung aufgebaut.")
            try:
                await client.get_services()
            except Exception:  # pylint: disable=broad-except
                pass

            self._active_notifications = []
            try:
                # Notifications aktivieren
                for uuid in notify_uuids:
                    try:
                        await client.start_notify(uuid, self._handle_notification)
                        self._active_notifications.append(uuid)
                        self.get_logger().info(f"Notifications aktiv auf {uuid}")
                    except Exception as exc:  # pylint: disable=broad-except
                        self.get_logger().warn(
                            f"Kann Notify auf {uuid} nicht starten: {exc}"
                        )

                # optionales Registrierungsframe
                if registration_payload:
                    await self._write_to_all(client, write_uuids, registration_payload)
                    await asyncio.sleep(0.5)

                # Polling-Schleife
                while rclpy.ok() and client.is_connected:
                    await self._write_to_all(client, write_uuids, query_payload)
                    await asyncio.sleep(query_interval)

            finally:
                # Nur die tatsächlich aktivierten wieder stoppen
                for uuid in list(self._active_notifications):
                    try:
                        await client.stop_notify(uuid)
                    except Exception:  # pylint: disable=broad-except
                        pass
                self._active_notifications = []

    async def _write_to_all(
        self, client: BleakClient, uuids: List[str], payload: bytes
    ):
        for uuid in uuids:
            try:
                # Viele BMS kommen mit response=False klar; falls nötig, auf True ändern.
                await client.write_gatt_char(uuid, payload, response=False)
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(f"Write auf {uuid} fehlgeschlagen: {exc}")

    def _handle_notification(self, uuid: str, data: bytearray):
        payload = bytes(data)

        ack = interpret_ack(payload)
        if ack:
            status_hex = f"0x{ack['status']:02x}"
            if ack["status"] == 0x00:
                log_func = self.get_logger().debug
            else:
                log_func = self.get_logger().warn
            log_func(f"ACK von {uuid}: cmd=0x{ack['ack_for']:02x} status={status_hex}")
            return

        status = parse_status_frame(payload)
        if status:
            self._publish_status(uuid, payload, status)
        else:
            self.get_logger().debug(
                f"Unbekannter Notify von {uuid}: {hx(payload)}  ({list(payload)})"
            )

    def _publish_status(self, uuid: str, payload: bytes, status: dict):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.voltage = float(status["total_voltage"])
        msg.current = float(status["current"])
        msg.temperature = float(status["cell_temp"])
        msg.cell_voltage = [float(v) for v in status["cell_voltages"]]
        msg.cell_temperature = [float(status["cell_temp"])]
        msg.present = True
        msg.location = "LiTime_BMS"
        msg.serial_number = self.get_parameter("mac").get_parameter_value().string_value

        msg.charge = float(status["remaining_ah"]) * 3600.0
        msg.capacity = float(status["full_capacity_ah"]) * 3600.0
        msg.design_capacity = msg.capacity
        msg.percentage = float(status["soc"]) / 100.0

        battery_state_code = status["battery_state"]
        if battery_state_code == "0001":
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif battery_state_code == "0000":
            msg.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                if msg.current < 0.0
                else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            )
        elif battery_state_code == "0004":
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

        protection_active = int(status["protection_state"], 16) != 0
        failure_active = int(status["failure_state"], 16) != 0
        if protection_active or failure_active:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        self._battery_pub.publish(msg)

        raw_msg = String()
        raw_msg.data = f"{uuid} {hx(payload)}"
        self._raw_pub.publish(raw_msg)

def main():
    rclpy.init()
    node = LiTimeBMSNode()
    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt – Node wird beendet.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
