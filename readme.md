This project is dedicated to the data-logging device for the rocky helmet. 

Upon a button press, the state machine will transition from rest to logging. It will begin reading acceleration data from an MPU6050 into a DMA, and upon filling the DMA it will commence flashing data to the onboard sd card. 

This is quite a basic fundamental task for an embedded system engineer. 

However, done well with good object oriented principles, this project can form the foundation for wide ranging future projects, and contains all the essential elements for much more advanced behaviour. 

We have a state machine. We have some sort of data processing. We have inputs and outputs. 

Done well, any one of these components can be substituted depending on future needs of a project. 

As reference, I have been closely following the recommendations made in "Making Embedded Systems by Elecia White.