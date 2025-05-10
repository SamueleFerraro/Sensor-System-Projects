# Homework 7 – Temperature Reading via I2C and Periodic UART Transmission

Reads the temperature from an LM75 sensor over I²C using an interrupt routine to capture all 11 bits (including sign). Data transfer is handled with DMA, and the value is sent to a remote terminal every 1 second.
