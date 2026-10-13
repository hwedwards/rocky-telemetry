This project is dedicated to the data-logging device for the rocky helmet. 

Upon a button press, the state machine will transition from rest to logging. It will begin reading acceleration data from an MPU6050 into a DMA, and upon filling the DMA it will commence flashing data to the onboard sd card. 

This is quite a basic fundamental task for an embedded system engineer. 

However, done well with good object oriented principles, this project can form the foundation for wide ranging future projects, and contains all the essential elements for much more advanced behaviour. 

We have a state machine. We have some sort of data processing. We have inputs and outputs. 

Done well, any one of these components can be substituted depending on future needs of a project. 

As reference, I have been closely following the recommendations made in "Making Embedded Systems by Elecia White.

## Logged IMU format

The MPU6050 FIFO and `log.bin` use 12-byte records: six signed 16-bit values
in big-endian order, `ax, ay, az, gx, gy, gz`. Temperature is not logged.
`FIFO_EN` (register `0x23`) is `0x78`: bits 6, 5, 4 enable the gyro axes and
bit 3 enables the accelerometer; temperature bit 7 is clear. Enabled data
enters the FIFO in ascending register order, so gyro X starts at byte 6.
See the manufacturer's [register map, sections 4.7 and 4.33](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf).

The logger drains whole records and writes buffers of 512 records (6,144 bytes,
twelve SD sectors). Storage writes the raw bytes unchanged; the diagnostic
decoder applies gyro bias and sets the absent temperature field to zero.
Direct register reads still use the sensor's separate 14-byte measurement
layout, which includes temperature. Do not interpret the FIFO temperature
placeholder as a temperature measurement.

Files recorded by the earlier `0xF8` configuration contain temperature and
need a 14-byte decoder; this fix does not convert existing logs.

See [FIFO regression checks](tests/README.md) for hardware-free validation.
