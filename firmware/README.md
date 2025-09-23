# Firmware notes

## Actuator ESP32 partition scheme

Si al arrancar el ESP32 ves mensajes como:

```
E (53) flash_parts: partition 3 invalid - offset 0x210000 size 0x200000 exceeds flash chip size 0x400000
E (53) boot: Failed to verify partition table
```

significa que la tabla de particiones con la que se compiló el firmware es para un chip de 8 MB, pero el actuador tiene 4 MB de flash.

Para corregirlo y volver al funcionamiento normal:

1. Copia el archivo [`firmware/actuator_partitions_4mb.csv`](actuator_partitions_4mb.csv) al directorio de particiones de tu instalación del ESP32 (Arduino IDE o PlatformIO).
2. Selecciona esa tabla de particiones al compilar (por ejemplo usando `arduino-cli --build-property build.partitions=actuator_partitions_4mb`).
3. Flashea nuevamente el firmware del actuador.

La tabla define dos particiones OTA de 1.9 MB cada una y se ajusta exactamente al chip de 4 MB, evitando el error de verificación y permitiendo que el código arranque normalmente.

Además, el firmware ahora imprime en el monitor serie la capacidad detectada del chip y la partición activa, lo que ayuda a verificar que se cargó la tabla correcta.
