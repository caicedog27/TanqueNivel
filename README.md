# Sistema de Control de Niveles Industrial

Este repositorio contiene el firmware para tres tipos de nodos ESP32 y un
servidor Python basado en FastAPI.  Se ha reorganizado la estructura para
facilitar un despliegue industrial donde cada sensor ultrasónico dispone de su
propio ESP32 y el actuador de la bomba funciona de forma independiente.

## Estructura

```
firmware/
  master_node/       # Puente WiFi ↔ ESP‑NOW
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

1. Ajuste las direcciones MAC y credenciales WiFi en cada firmware.
2. Compile y cargue cada sketch en su correspondiente ESP32.
3. Ejecute el servidor con:
   ```bash
   uvicorn server.main:app --reload --host 0.0.0.0 --port 8000
   ```

El servidor expone `/ws/bridge` para el nodo maestro y `/ws/client` para la
interfaz web.

