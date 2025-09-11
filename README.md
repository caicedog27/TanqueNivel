# Sistema de Control de Niveles Industrial

Este repositorio contiene el firmware para dos tipos de nodos ESP32 y un
servidor Python basado en FastAPI. Cada sensor ultrasónico dispone de su
propio ESP32 conectado por WiFi al servidor, y el actuador de la bomba
funciona de forma independiente también vía WiFi.

## Estructura

```
firmware/
  actuator_node/     # Control del relé con temporización y tren de pulsos
  sensor_node_single/# Firmware genérico para un sensor ultrasónico
server/
  main.py            # Servidor FastAPI con WebSockets y API REST
  static/            # Interfaz web
```

## Características destacadas

- **Filtrado de ruido**: los sensores usan un buffer circular y mediana para
  eliminar lecturas espurias.
- **Persistencia**: calibraciones y parámetros se guardan en NVS (Preferences).
- **Reconexión y Heartbeats**: los nodos envían pulsos de vida y el servidor
  supervisa los tiempos de espera.
- **Seguridad del relé**: el actuador incluye TTL para apagar la bomba si no
  recibe comandos, además de un relé de pulsos configurable.
- **Servidor modular**: FastAPI gestiona la comunicación con los nodos y
  clientes web, guardando estado en archivos JSON.

## Uso

1. Ajuste las credenciales WiFi y la IP del servidor en cada firmware.
2. Compile y cargue cada sketch en su correspondiente ESP32.
3. Ejecute el servidor con:
   ```bash
   uvicorn server.main:app --reload --host 0.0.0.0 --port 8000
   ```

Cada nodo se conecta a `/ws/board/{ID}` (por ejemplo `/ws/board/SENS` o
`/ws/board/ACT`) y la interfaz web usa `/ws/client`.

