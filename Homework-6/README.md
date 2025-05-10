# Homework 6 – Periodic Sensor Acquisition with Timer-Triggered ADC and UART Output

- **Part a**: Samples LDR resistance every 1 ms using timer-triggered ADC with circular DMA. Samples are split into two halves to allow processing between acquisitions. The average is computed every 1 s, converted to lux, and sent to a remote terminal.  
- **Part b**: Acquires voltages from a potentiometer, temperature sensor, and Vref every 1 s using timer-triggered ADC with circular DMA, and sends the values to a remote terminal.

