# MAX7219_STM32_ANT_GAME_LIFE

This two in one project langton's ant and the Conway's Game of Life.  

## Material list

1 blue pill STM32F103C8 or STM32F1 family microcontroller  
1 8x8 LED Matrix Drived with MAX7219 (optional)  

## MCU Pinout 

![Schematics](/Pictures/STM32Cube.PNG)

RCC_OSC_IN & RCC_OSC_OUT 	= 8MHz Oscillator  
LED 				= Led for indicator  
SPI1\* and CS pins 		= MAX7219  
SYS_JT\*			= debug  

## Little guide of usage 

The 1st function called is init_map, it's take 2 arguments :  
- The 1st one is a pointer of a St_Ant structure  
- The 2nd one is the structure map choosen (uint8_t)  
  
The 2nd function called is HAL_TIM_Base_Start_IT :  
- The only one parameter is a pointer of a TIM_HandleTypeDef  
  
The 3rd function called is Maxim_Init  
- The only one parameter is a pointer of a SPI_HandleTypeDef  
  
The 4th function called is Maxim_Map :  
- The 1st parameter is a pointer of a SPI_HandleTypeDef  
- The 2nd parameter is a pointer to the map

The 5th function called are life_game or Ant one of it is commented :  
-The only one parameter is map of to the map
  
  
## Future features and bug to resolve

- [ ] Add other init structures for maps like Oscillators and Spaceships (Named in french in the code :)  

## Libararies and programs used to init project

SMT32CUBEMX was used to initialise the project code, please open the .ioc file to find my microcontroller configuration.  
All libraries used in this project are from STM32CUBEMX from STMicroelectronics.
