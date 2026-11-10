# 2026-11-09 · Prueba 08, flexión y extensión de la rodilla FL

Nota: [docs/bitacora/08_flexion_rapida.md](../../docs/bitacora/08_flexion_rapida.md).

Corrida de las 18:27 (`pc/resultados_flexion/20261109_182700/`), copiada tal cual. Fuente de 6.0 V,
`esp32_principal` del 9 de noviembre (con `DIAG=1`), `datos/2026-11-09_patas/pata_fl.json`.

| Archivo | Qué es |
|---|---|
| `meta.json` | Cómo se corrió: pata, límites, pulsos de los extremos, velocidades, la línea `?` de la ESP32 |
| `corridas.csv` | Una fila por velocidad: muestras, comandos, líneas perdidas (0 en todas) y si llegó |
| `muestras.csv` | El cable verde cada ~2 ms (mV), por velocidad; t en s desde que empezó la rampa |
| `comandos.csv` | El pulso aplicado cada 20 ms (µs), por velocidad |
| `metricas.csv` | Las medidas de cada flexión y extensión |
| `resumen.md`, `trazas.png`, `flexion.png` | Lo que generó `flexion.py` |

Para volver a analizarla: `python pc\flexion.py --reanalizar datos\2026-11-09_flexion_fl`
