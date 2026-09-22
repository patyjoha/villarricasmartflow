# Villarrica Smartflow: Sistema IoT de Monitoreo Urbano

Repositorio oficial de los prototipos del proyecto **Villarrica Smartflow**, un sistema IoT diseñado para mejorar la gestión del tráfico y la accesibilidad peatonal en el microcentro de Villarrica.

## 📁 Estructura del Repositorio

Este proyecto se divide en dos nodos principales desarrollados sobre microcontroladores ESP32:

*   **`/villarrica-smartflow` (Nodo A - Flujo Urbano):** Simulación del sensor encargado de medir la densidad del flujo peatonal y vehicular. Cuenta detecciones y transmite la telemetría a la plataforma IoT para su análisis.
*   **`/villarrica-smartflow-nodo-b` (Nodo B - Semáforo Accesible):** Simulación de un semáforo adaptativo. Reduce la espera vehicular si la calzada está libre e incorpora funciones de accesibilidad (pulsador y señales acústicas) para personas con discapacidad visual.

## 🛠️ Tecnologías Utilizadas
*   **Hardware (Simulado):** ESP32, Sensores PIR (simulando radar Doppler y detectores de presencia), LEDs, Zumbadores.
*   **Software / Entorno:** VS Code con PlatformIO y Wokwi.
*   **Comunicaciones / Plataforma:** MQTT, ThingsBoard (para el dashboard y la analítica).

## 🚀 Cómo ejecutar las simulaciones
Cada carpeta incluye su propio archivo `diagram.json` para ejecutar la prueba directamente en **Wokwi**.
1. Abre la carpeta del nodo que deseas probar en VS Code.
2. Asegúrate de tener instalada la extensión de Wokwi.
3. Abre el archivo `diagram.json` e inicia la simulación.
