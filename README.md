# NES-Video-Game
A NES game controller to FPGA board adapter that is used to play a two player tug of war game programmed on Quartus Prime

The NES adapter sends and receives the appropriate signals from the NES controller and then using Finite State Machines (FSM) and logic elements, the adapter is able to translate the electrical signals into their corresponding button states (A, B, Start, Select etc...). 

Using more FSMs, logic elements and Linear Feedback Shift Registers (LFSR), the FPGA is able to choose and select the correct BRAMS to send to the VGA depending on which button each user presses. 

The game is a two player tug of war game.
# Rules 
- Both players start at the middle of the arena 
- Two random buttons, one for each player, will be generated and displayed on the screen
- If a player presses the button indicated on the screen, the rope will move closer to their side; however each correct button press by the opposing player will move the rope closer to the opponent's side 
- Once the rope is all the way on the side of one player, the game will end  
