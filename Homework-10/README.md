# Homework 10 – Keypad Scanning and Encoder RPM Measurement

- **Part a**: Scans a 4×4 keypad using a 5 ms timer interrupt and debounces inputs with multiple samples per column. Flags new key presses and transmits them in batches over UART using DMA.

- **Part b**: Reads a rotary encoder's position and computes the rotation speed in RPM. A timer provides a fixed timebase, and the result is sent to the PC via UART using DMA.
