"""PLC actuator integration for Siemens S7-1200 controllers.

This module provides a small wrapper around the python-snap7 client in order
to integrate the PLC based actuator with the existing control server.  The goal
is to provide a resilient and easy to use interface for writing basic commands
such as the pump state and duty cycle values.
"""
from __future__ import annotations

import asyncio
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import snap7  # type: ignore
    from snap7.exceptions import Snap7Exception  # type: ignore
    from snap7.util import set_bool, set_dint  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    snap7 = None  # type: ignore

    class Snap7Exception(Exception):  # type: ignore
        """Fallback exception when python-snap7 is not available."""

    def set_bool(*_: object, **__: object) -> None:  # type: ignore
        raise Snap7Exception("python-snap7 no está instalado")

    def set_dint(*_: object, **__: object) -> None:  # type: ignore
        raise Snap7Exception("python-snap7 no está instalado")


class PlcActuatorError(RuntimeError):
    """Error de alto nivel al operar el PLC."""


@dataclass
class PlcAddress:
    db: int
    start: int
    bit: Optional[int] = None


@dataclass
class PlcActuatorConfig:
    host: str = ""
    rack: int = 0
    slot: int = 1
    pump_address: PlcAddress = field(default_factory=lambda: PlcAddress(db=1, start=0, bit=0))
    duty_on_address: PlcAddress = field(default_factory=lambda: PlcAddress(db=1, start=2))
    duty_off_address: PlcAddress = field(default_factory=lambda: PlcAddress(db=1, start=6))
    heartbeat_address: Optional[PlcAddress] = field(default_factory=lambda: PlcAddress(db=1, start=10))
    heartbeat_interval_s: float = 5.0

    def heartbeat_enabled(self) -> bool:
        return self.heartbeat_address is not None


class S7ActuatorClient:
    """Cliente asíncrono sencillo para escribir comandos al PLC."""

    def __init__(self, cfg: PlcActuatorConfig):
        self._cfg = cfg
        self._client: Optional["snap7.client.Client"] = None  # type: ignore
        self._lock = threading.Lock()

    @property
    def enabled(self) -> bool:
        return snap7 is not None and bool(self._cfg.host)

    def update_config(self, cfg: PlcActuatorConfig) -> None:
        with self._lock:
            self._cfg = cfg
            if self._client is not None:
                try:
                    self._client.disconnect()
                except Exception:
                    pass
                self._client = None

    async def set_pump(self, on: bool) -> None:
        await asyncio.to_thread(self._set_pump_sync, on)

    async def set_duty(self, on_ms: int, off_ms: int) -> None:
        await asyncio.to_thread(self._set_duty_sync, on_ms, off_ms)

    async def heartbeat(self) -> None:
        await asyncio.to_thread(self._heartbeat_sync)

    # ---------- Internal helpers ----------
    def _ensure_connection(self) -> "snap7.client.Client":  # type: ignore
        if not self.enabled:
            raise PlcActuatorError("PLC S7-1200 deshabilitado o snap7 no disponible")
        with self._lock:
            if self._client is None:
                self._client = snap7.client.Client()  # type: ignore
            client = self._client
            try:
                if not client.get_connected():
                    client.connect(self._cfg.host, self._cfg.rack, self._cfg.slot)
            except Exception as exc:  # pragma: no cover - conexión fallida
                try:
                    client.disconnect()
                except Exception:
                    pass
                self._client = None
                raise PlcActuatorError(f"No se pudo conectar al PLC: {exc}") from exc
            return client

    def _set_pump_sync(self, on: bool) -> None:
        client = self._ensure_connection()
        addr = self._cfg.pump_address
        if addr.bit is None:
            raise PlcActuatorError("La dirección de la bomba debe incluir el bit")
        try:
            data = bytearray(client.db_read(addr.db, addr.start, 1))
        except Exception as exc:
            raise PlcActuatorError(f"Lectura DB bomba falló: {exc}") from exc
        try:
            set_bool(data, 0, addr.bit, on)
        except Exception as exc:  # pragma: no cover - utilidades snap7
            raise PlcActuatorError(f"No se pudo preparar el dato de la bomba: {exc}") from exc
        try:
            client.db_write(addr.db, addr.start, data)
        except Exception as exc:
            raise PlcActuatorError(f"Escritura DB bomba falló: {exc}") from exc

    def _set_duty_sync(self, on_ms: int, off_ms: int) -> None:
        client = self._ensure_connection()
        on_addr = self._cfg.duty_on_address
        off_addr = self._cfg.duty_off_address
        try:
            on_buf = bytearray(4)
            off_buf = bytearray(4)
            set_dint(on_buf, 0, int(on_ms))
            set_dint(off_buf, 0, int(off_ms))
        except Exception as exc:  # pragma: no cover
            raise PlcActuatorError(f"No se pudo preparar el dato de duty: {exc}") from exc
        try:
            client.db_write(on_addr.db, on_addr.start, on_buf)
            client.db_write(off_addr.db, off_addr.start, off_buf)
        except Exception as exc:
            raise PlcActuatorError(f"Escritura duty falló: {exc}") from exc

    def _heartbeat_sync(self) -> None:
        client = self._ensure_connection()
        addr = self._cfg.heartbeat_address
        if not addr:
            # Sólo verificar la conexión
            return
        try:
            payload = bytearray(4)
            set_dint(payload, 0, int(time.time()))
            client.db_write(addr.db, addr.start, payload)
        except Exception as exc:
            raise PlcActuatorError(f"Heartbeat PLC falló: {exc}") from exc


__all__ = ["PlcActuatorError", "PlcActuatorConfig", "PlcAddress", "S7ActuatorClient"]
