# UPCR TEC Module Firmware

## Workflow 
UART: 
* wait for communication
* store incoming data into DMA rx_buffer
* once the rx_buffer is full, trigger an interrupt to raise the ```new_rx_data_flag``` 
* copy rx_buffer data into another variable to avoid getting it overwritten
* [do stuff depending on the protocol you define]
    * in my example: T 62 0 0 starts PID temperature control to 62 deg C
    * in my example: S 0  0 0 stops the PID temperature control
 

ADC:  
* If PID control is enabled, enable timer 16
* Timer 16 sets interrupts to triger ADC sampling
* ADC read all 4 temperature sensor channels and stores data into DMA buffer
* when the DMA buffer is full, trigger an interrupt to raise the ``` dma_adc_status_flag```
* use the corresponding data as ```Input``` for your PID loop

PID: 
> because the chip has a very small flash memory, we cannot use floating point maths to run our PID loop. 
* Convert temperature received from UART to 100th od degrees 
* find the equivalent ADC value in the temperature lookup table
    * the temperature lookup table is generated using NTC equations with a python script in ```temperature lookup table generator```
* use this equivalent as ```Setpoint``` in the PID loop
* [tune PID loop coefficients in ```NTC.h```] 

## Tools, Tutorials and References
* [STM32 CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
* [Known bug](https://community.st.com/s/question/0D50X0000BbMcnpSQC/dma-adc-doesnt-work) 
* [Tutorials](https://letanphuc.net/)