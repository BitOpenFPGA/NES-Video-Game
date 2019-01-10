/*
module top (CLOCK_50, KEY, SW, LEDR);
	input CLOCK_50;
	input [3:0] KEY;
	input [9:0] SW; 
	output [9:0] LEDR; 
	
	gamemaster gm (CLOCK_50, KEY[0], SW[0], SW[1], SW[2], SW[3], SW[4], SW[5], SW[6], SW[7], LEDR[0], LEDR[1], LEDR[2], LEDR[3], LEDR[4], LEDR[5], LEDR[6], LEDR[7]); 
	
endmodule 

module gamemaster(clock, resetn, first, start, player, pause, buttonpress, doneleft, doneright, donemove, ld_screen, ld_player, ld_pause, ld_p1, ld_p2, startleft, startright, startmove); 
	input clock, resetn; 
	input first, start, player, pause, buttonpress, doneleft, doneright, donemove; 
	
	output reg ld_screen, ld_player, ld_pause, ld_p1, ld_p2, startleft, startright, startmove; 
	
	
	reg [5:0] current_state, next_state; 
	
	localparam  S_BG					= 5'd0,
					S_START				= 5'd1,
					S_PLAYER				= 5'd2,
					S_PAUSE 				= 5'd3,
					S_WAIT				= 5'd4, 
					S_LOAD				= 5'd5, 
					S_DRAWLEFT		= 5'd6, 
					S_DRAWRIGHT		= 5'd7,
					S_MOVE				= 5'd8; 
					
	always @(*)
	begin: state_table 
	
		case (current_state) 
			S_BG: begin 
				if(first == 1) next_state = S_START; 
				else next_state = S_BG; 
			end
			S_START: begin
				if(start == 1) next_state = S_PLAYER; 
				else next_state = S_START; 
			end
			S_PLAYER: begin
				if(player == 1) next_state = S_PAUSE; 
				else next_state = S_PLAYER; 
			end 
			S_PAUSE: begin
				if(pause == 1) next_state = S_WAIT; 
				else next_state = S_PAUSE; 
			end
			S_WAIT: begin
				if(buttonpress == 1) next_state = S_LOAD; 
				else next_state = S_WAIT;
			end
			S_LOAD: begin 
				 next_state = S_DRAWLEFT; 
			end 
			S_DRAWLEFT: begin
				if(done == 1) next_state = S_CHOOSERIGHT; 
				else next_state = S_DRAWLEFT; 
			end 
			S_DRAWRIGHT: begin
				if(done == 1) next_state = S_MOVE; 
				else next_state = S_DRAWRIGHT; 
			end 
			S_MOVE: begin 
				if(donemove == 1) next_state = S_WAIT; 
				else next_state = S_MOVE; 
			end 
			default: next_state = S_BG; 
		endcase 
	end
	
	always @(*) 
	begin: enable_signals 
		ld_screen = 1'b0; 
		ld_player = 1'b0; 
		ld_pause = 1'b0; 
		ld_p1 = 1'b0;
		ld_p2 = 1'b0; 
		startleft = 1'b0; 
		startright = 1'b0; 
		startmove = 1'b0; 
		
		case (current_state) 
			S_START: begin
				ld_screen = 1'b1; 
			end
			S_PLAYER: begin
				ld_player = 1'b1; 
			end
			S_PAUSE: begin
				ld_pause = 1'b1; 
			end
			S_LOAD: begin
				ld_p1 = 1'b1; 
				ld_p2 = 1'b1; 
			end 
			S_DRAWLEFT: begin
				startleft = 1'b1; 
			end 
			S_DRAWRIGHT: begin
				startright = 1'b1; 
			end 
			S_MOVE: begin
				startmove = 1'b1; 
			end 
		endcase 
	end
	
	always @(posedge clock) 
	begin: state_FFs 
		if(!resetn) current_state <= S_BG; 
		else current_state <= next_state; 
	end
	
endmodule 

*/


module gaymemastercontrol(clock, resetn, done, resetit, leftEn, rightEn); 
	input clock, resetn, done;  
	output reg leftEn;
	output reg rightEn;
	output reg resetit; 
	
	reg [5:0] current_state, next_state; 
	
	localparam  S_DRAWL			= 5'd0, 
					S_RESET1			= 5'd1,
					S_DRAWR			= 5'd2,
					S_RESET2			= 5'd3;
					
	always @(*)
	begin: state_table 
	
		case (current_state) 
			S_DRAWL: begin
				if((done == 1)) next_state = S_RESET1; 
				else next_state = S_DRAWL; 
			end 
			S_RESET1: begin
				next_state=S_DRAWR;
			end
			S_DRAWR: begin
				if((done == 1)) next_state = S_RESET2; 
				else next_state = S_DRAWR; 
			end 
			S_RESET2: begin
				next_state=S_DRAWL;
			end
			default: next_state = S_DRAWL; 
		endcase 
	end
	
	always @(*) 
	begin: enable_signals  
		resetit=1'b0;
		leftEn=1'b0;
		rightEn=1'b0;
		case (current_state) 
			S_DRAWL: begin
				 leftEn= 1'b1;
			end
			S_RESET1: begin
				resetit = 1'b1;
			end
			S_DRAWR: begin
				rightEn = 1'b1;
			end
			S_RESET2: begin
				resetit = 1'b1;
			end
		endcase 
	end
	
	always @(posedge clock) 
	begin: state_FFs 
		if(!resetn) current_state <= S_DRAWL; 
		else current_state <= next_state; 
	end
	
endmodule 
//leftSel might have accidentally been filled with right sel


module gaymemasterdata (clock, resetn, SW, le, re, button1, button2, finalbutton);
	input clock, resetn, le, re;
	input [2:0] button1, button2; 
	input [9:0] SW;
	output reg [3:0] finalbutton; 
	//have the reg here and count score always here

	
	always@(posedge clock) begin
			
		if(!resetn) begin
			finalbutton <= 4'b0000; 
		end
		else begin
			if(le) begin
				finalbutton <= {1'b0, button1}; 
			end
			else if (re) begin
				finalbutton <= {1'b1, button2}; 
			end
		end
	end
	
	
endmodule


module fill
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		KEY,		
		LEDR,
		// On Board Keys
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		SW,
		GPIO_0,
		GPIO_1
	);

	input			CLOCK_50;				//	50 MHz
	input	[3:0]	KEY;	
	input [9:0] SW; 
	input [35:0] GPIO_0; 
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[7:0]	VGA_R;   				//	VGA Red[7:0] Changed from 10 to 8-bit DAC
	output	[7:0]	VGA_G;	 				//	VGA Green[7:0]
	output	[7:0]	VGA_B;   				//	VGA Blue[7:0]
	output [9:0] LEDR;
	output [35:0] GPIO_1; 
	
	wire resetn;
	assign resetn = KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
 
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn, done, resetit;
	wire [2:0] toBePressed;
	wire [2:0] toBePressed2;
	wire [3:0] finalbutton; 
	wire go;
	wire go2;
	wire leftEn, rightEn;
	
	wire [7:0] button, button2;
	
	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "background V2.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.
	
	gaymemastercontrol GOD (CLOCK_50, resetn, done, resetit, leftEn, rightEn);//toBePressedFinal
	gaymemasterdata PLEASE (CLOCK_50, resetn, SW, leftEn, rightEn, toBePressed, toBePressed2, finalbutton);
	
	
	colours WORK(CLOCK_50, KEY, resetit, finalbutton, x, y, colour, writeEn, done); 
	//module top (KEY, CLOCK_50, go, LEDR, toBePressed);
	top GZUS(KEY, CLOCK_50, go, LEDR[2:0], toBePressed);
	top2 ASJD (KEY, CLOCK_50, go2, LEDR[5:3], toBePressed2);
	//module buttonpress (KEY, button, toBePressed, clock, go);
	
	NESReader PRAYERS (CLOCK_50, GPIO_0[1:0], KEY, button, GPIO_1[1:0]);
	NESReader DJKHALED (CLOCK_50, GPIO_0[3:2], KEY, button2, GPIO_1[3:2]); 
	
	buttonpress bp(KEY, button, toBePressed, CLOCK_50, go);
	buttonpress2 bp2(KEY, button2, toBePressed2, CLOCK_50, go2);
	
endmodule




module colours (clock, KEY, resetit, enable, x, y, colour, writeEn, drawn);
	input clock, resetit;
	input [3:0] KEY, enable; 
	
	output reg [2:0] colour; 
	output reg [7:0] x;
	output reg [6:0] y; 
	output reg writeEn, drawn; 
	
	wire [2:0] ctemp;
	wire [7:0] xtemp;
	wire [6:0] ytemp;
	wire wtemp, done; 
	
	choose buttons (clock, KEY[0], resetit, enable, ctemp, xtemp, ytemp, wtemp, done);
	
	always @(*) begin
		if(!KEY[0]) begin
			x <= 8'b00000000;
			y <= 7'b0000000; 
			colour <= 3'b000; 
			writeEn <= 1'b0; 
			drawn <= 1'b0; 
		end
		else begin
			if(done == 1'b0) begin
				x <= xtemp;
				y <= ytemp; 
				colour <= ctemp; 
				writeEn <= wtemp; 
				drawn <= 1'b0; 
			end
			else begin
				drawn <= 1'b1; 
				writeEn <= 1'b0;  
			end
		end
	
	end
	
endmodule



module choose (clock, resetn, resetit , enable, colour, x, y, writeEn, done); 
	input clock, resetit; 
	input [3:0] enable; 
	input resetn;
	output reg [2:0] colour;
	output reg [7:0] x;
	output reg [6:0] y;
	output reg writeEn, done;
	
	reg [11:0] c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16; 
	wire [2:0] col1, col2, col3, col4, col5, col6, col7, col8, col9, col10, col11, col12, col13, col14, col15, col16; 
	reg [2:0] colourtemp; 
	
	reg le;
	wire ld;
	wire [7:0] lx;
	wire [6:0] ly; 
	wire [11:0] lc;  
	
	loopCommand coordinates (clock, le, resetn, resetit, lc, lx, ly, ld); 
	
	leftRight	p1R	(c1, clock, 3'b000, 1'b0, col1); 
	leftLeft		p1L	(c2, clock, 3'b000, 1'b0, col2); 
	leftDown		p1D	(c3, clock, 3'b000, 1'b0, col3); 
	leftUp		p1U	(c4, clock, 3'b000, 1'b0, col4); 
	leftStrt 	p1ST	(c5, clock, 3'b000, 1'b0, col5); 
	leftSel 		p1SL	(c6, clock, 3'b000, 1'b0, col6); 
	leftB 		p1B	(c7, clock, 3'b000, 1'b0, col7); 
	leftA 		p1A 	(c8, clock, 3'b000, 1'b0, col8);
	
	rightRight	p2R	(c9, clock, 3'b000, 1'b0, col9); 
	rightLeft	p2L	(c10, clock, 3'b000, 1'b0, col10); 
	rightDown	p2D	(c11, clock, 3'b000, 1'b0, col11); 
	rightUp		p2U	(c12, clock, 3'b000, 1'b0, col12); 
	rightStrt	p2ST	(c13, clock, 3'b000, 1'b0, col13); 
	rightSel		p2SL	(c14, clock, 3'b000, 1'b0, col14); 
	rightB 		p2B	(c15, clock, 3'b000, 1'b0, col15);
	rightA		p2A	(c16, clock, 3'b000, 1'b0, col16); 
	
	//so like this shit is defined in the wrong order but like
	//it's way more work to fix this than just like change a few values later on
	
	localparam leftR = 4'b0000, leftL = 4'b0001, leftD = 4'b0010, leftU = 4'b0011, leftST = 4'b0100, leftSL = 4'b0101, leftB = 4'b0110, leftA = 4'b0111, 
				  rightR = 4'b1000, rightL = 4'b1001, rightD = 4'b1010, rightU = 4'b1011, rightST = 4'b1100, rightSL = 4'b1101, rightB = 4'b1110, rightA = 4'b1111; 
	
	
	always @ (*) begin
		if(!resetn) begin
			colour <= 3'b000; 
			x <= 8'b00000000;
			y <= 7'b0000000;
			writeEn <= 1'b0;
			done <= 1'b0; 
			le <= 1'b0; 
		end
		else begin
			if(ld == 0) begin
				if(enable[3] == 1'b0) begin
					colour <= colourtemp; 
					x <= lx + 5'b11011;		//have to add x,y (starting) coordinates
					y <= ly + 3'b111; 
					writeEn <= 1'b1; 
					done <= 1'b0; 
					le <= 1'b1; 
				end
				if(enable [3] == 1'b1) begin
					colour <= colourtemp;
					x <= lx + 7'b1010000;		//have to add x,y (starting) coordinates
					y <= ly + 3'b111; 
					writeEn <= 1'b1; 
					done <= 1'b0; 
					le <= 1'b1; 
				end
			end
			else begin
				done <= 1'b1; 
				writeEn <= 1'b0; 
				le <= 1'b0;  
			end
		end
	
	end
	
	
	always @ (posedge clock) begin //this waits
		if(!resetn) begin 
			c1 <= 12'b000000000000;
			c2 <= 12'b000000000000; 
			c3 <= 12'b000000000000; 
			c4 <= 12'b000000000000; 
			c5 <= 12'b000000000000; 
			c6 <= 12'b000000000000; 
			c7 <= 12'b000000000000; 
			c8 <= 12'b000000000000; 
			c9 <= 12'b000000000000; 
			c10 <= 12'b000000000000; 
			c11 <= 12'b000000000000; 
			c12 <= 12'b000000000000; 
			c13 <= 12'b000000000000; 
			c14 <= 12'b000000000000; 
			c15 <= 12'b000000000000; 
			c16 <= 12'b000000000000; 
		end 
		else begin 
			case (enable)
				leftR: begin 
					c1 <= lc;
				end
				leftL: begin 
					c2 <= lc;
				end 
				leftD: begin 
					c3 <= lc; 
				end 
				leftU: begin 
					c4 <= lc; 
				end
				leftST: begin
					c5 <= lc; 
				end
				leftSL: begin
					c6 <= lc; 
				end
				leftB: begin
					c7 <= lc; 
				end 
				leftA: begin
					c8 <= lc; 
				end
				
				
				rightR: begin 
					c9 <= lc; 
				end
				rightL: begin  
					c10 <= lc; 
				end 
				rightD: begin
					c11 <= lc; 
				end 
				rightU: begin
					c12 <= lc; 
				end
				rightST: begin
					c13 <= lc; 
				end
				rightSL: begin
					c14 <= lc; 
				end
				rightB: begin
					c15 <= lc; 
				end 
				rightA: begin
					c16 <= lc; 
				end
			endcase 
		end
	end 	
	
	
	always @ (posedge clock) begin //this waits
		if(!resetn) begin 
			colourtemp <= 3'b000; 
		end 
		else begin 
			case (enable)
				leftR: begin 
					colourtemp <= col1; 
				end
				leftL: begin 
					colourtemp <= col2;
				end 
				leftD: begin 
					colourtemp <= col3;
				end 
				leftU: begin 
					colourtemp <= col4;
				end
				leftST: begin
					colourtemp <= col5;
				end
				leftSL: begin 
					colourtemp <= col6;
				end
				leftB: begin 
					colourtemp <= col7;
				end 
				leftA: begin
					colourtemp <= col8;
				end
				rightR: begin 
					colourtemp <= col9;
				end
				rightL: begin  
					colourtemp <= col10;
				end 
				rightD: begin
					colourtemp <= col11;
				end 
				rightU: begin 
					colourtemp <= col12;
				end
				rightST: begin
					colourtemp <= col13;
				end
				rightSL: begin
					colourtemp <= col14;
				end
				rightB: begin
					colourtemp <= col15;
				end 
				rightA: begin 
					colourtemp <= col16;
				end
			endcase 
		end
	end 	
	
endmodule 




module loopCommand (clock, enable, resetn, resetit, counter, counter_x, counter_y, done);
	input clock; 
	input enable; 
	input resetn, resetit;
	output reg [11:0] counter; 
	output reg [7:0] counter_x;
	output reg [6:0] counter_y;
	output reg done; 
	reg go;
	initial go=1'b0;
	
	always@(posedge clock, negedge resetn) begin
		if(resetn == 1'b0) begin
			counter <= 12'b000000000000; 
			counter_x <= 8'b00000000; 
			counter_y <= 7'b0000000; 
			done <= 1'b0; 
			go<=1'b0;
		end
		else begin
			if((counter < 12'b110100001010)&&(go==1'b1)) begin 
				if(done == 1'b0) begin 
					if(enable == 1'b1) begin
						counter_x <= counter_x + 1; 
						if(counter_x == 8'b0110100) begin 
							counter_x <= 7'b0000000; 
							counter_y <= counter_y + 1; 
						end
						counter <= counter + 1;
						go<=1'b0;	
					end
				end
			end	
			else if(go==1'b0) go<=1'b1;
			else begin  
				done <= 1'b1; 
				counter <= 12'b000000000000; 
				counter_x <= 8'b00000000; 
				counter_y <= 7'b0000000;
				go<=1'b0;
			end
			if(resetit == 1) begin
				done <= 1'b0; 
			end
		end
	end	
endmodule 

/*
module tester (CLOCK_50, KEY, GPIO_0, GPIO_1, LEDR); 
	input CLOCK_50;
	input [3:0] KEY;
	input [35:0] GPIO_0, GPIO_1; 
	output [9:0] LEDR; 
	
	wire 
	
	top topyy (KEY, CLOCK_50, go, LEDR, toBePressed);
	buttonpress (KEY, button, toBePressed, CLOCK_50, go);
	NESReader asdf (CLOCK_50, GPIO_0, KEY, button, GPIO_1);
	
	
endmodule 
*/


module top (KEY, CLOCK_50, go, LEDR, toBePressed);
	input [3:0] KEY; 
	input go;
	input CLOCK_50;
	output [2:0] LEDR; 
	output [2:0] toBePressed;
	//output [6:0] HEX0; 
	
	wire [2:0] num; 
	reg [3:0] letter; 
	//so these are the random shits for the first one
	lfsr #(16'b1000010001001001) number1 (CLOCK_50, KEY[0], go, num[0]);
	lfsr #(16'b1010101011000101) number2 (CLOCK_50, KEY[0], go, num[1]);
	lfsr #(16'b1001110110101010) number3 (CLOCK_50, KEY[0], go, num[2]);
	
	assign LEDR [0] = num[0];
	assign LEDR [1] = num[1];
	assign LEDR [2] = num[2];
	
	assign toBePressed [0] = num[0];
	assign toBePressed [1] = num[1];
	assign toBePressed [2] = num[2];
	//assign toBePressed[3]=1'b1;
	//hex_decoder hex (letter, HEX0);
	
	
endmodule

 
module lfsr (clock, resetn, enable, num);
	input enable, clock, resetn;
	output reg num;
	parameter seed = 16'b1000010001001001;
   reg [15:0] out;
   wire linear_feedback;
	
   assign linear_feedback =  ~(out[12]^(out[15]^out[13]));
  
   always @(posedge clock, negedge resetn)
   if (!resetn) begin
     out <= seed;
	  num <= 1'b0; 
	end 
   else if (enable) begin
     out <= {out[14:0], linear_feedback}; 
	  num <= out[7]; 
   end 
  
endmodule 
//tie lfsr to done


module buttonpress (KEY, button, toBePressed, clock, go);
	input [3:0] KEY;
	input clock;
	input [7:0] button;
	input [2:0] toBePressed;
	output go;
	
	reg [2:0] currentState, nextState;
	localparam start=3'b000, first1=3'b001, first0=3'b011, last1=3'b101; 
	
	always@(*)
	begin: state_table
		case(currentState)
		start: begin 
				//if(button == 0) nextState = first1;//once they press down move to the next state
				
				if((toBePressed==3'b111)&&(button[7] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b110)&&(button[6] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b101)&&(button[5] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b100)&&(button[4] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b011)&&(button[3] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b010)&&(button[2] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b001)&&(button[1] == 0)) begin
						nextState = first1;
				end				
					else if((toBePressed==3'b000)&&(button[0] == 0)) begin
						nextState = first1;
				end
				
				
				else nextState = start;
		end
		first1: begin
				if((toBePressed==3'b111)&&(button[7] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b110)&&(button[6] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b101)&&(button[5] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b100)&&(button[4] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b011)&&(button[3] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b010)&&(button[2] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b001)&&(button[1] == 1)) begin
						nextState = last1;
				end				
					else if((toBePressed==3'b000)&&(button[0] == 1)) begin
						nextState = last1;
				end
				
				else nextState =  first1;//so once they've let go move to the next state
					//this also means that they've pressed and let go of the button once
		end
		last1: begin
				nextState = start; 
		end
		default : nextState = start; 
		endcase
	end
	
	always @(posedge clock)
    begin: state_FFs			
      if(KEY[0] == 1'b0)
         currentState <= start; // Should set reset state to state A
      else
			currentState <= nextState;
   end // state_FFS
	 
	assign go = (currentState == last1);
	
endmodule




module top2 (KEY, CLOCK_50, go, LEDR, toBePressed);
	input [3:0] KEY; 
	input go;
	input CLOCK_50;
	output [2:0] LEDR; 
	output [2:0] toBePressed;
	
	wire [2:0] num; 
	reg [3:0] letter; 
	//so these are the random shits for the first one
	lfsr #(16'b1000010001001001) number1 (CLOCK_50, KEY[0], go, num[0]);
	lfsr #(16'b1010101011000101) number2 (CLOCK_50, KEY[0], go, num[1]);
	lfsr #(16'b1001110110101010) number3 (CLOCK_50, KEY[0], go, num[2]);
	
	assign LEDR [0] = num[0];
	assign LEDR [1] = num[1];
	assign LEDR [2] = num[2];
	
	assign toBePressed [0] = num[0];
	assign toBePressed [1] = num[1];
	assign toBePressed [2] = num[2];

endmodule

 
module lfsr2 (clock, resetn, enable, num);
	input enable, clock, resetn;
	output reg num;
	parameter seed = 16'b1000010001001001;
   reg [15:0] out;
   wire linear_feedback;
	
   assign linear_feedback =  ~(out[12]^(out[15]^out[13]));
  
   always @(posedge clock, negedge resetn)
   if (!resetn) begin
     out <= seed;
	  num <= 3'b000; 
	end 
   else if (enable) begin//!!
     out <= {out[14:0], linear_feedback}; 
	  num <= out[7]; 
   end 
  
endmodule 
//tie lfsr to done



module buttonpress2 (KEY, button, toBePressed, clock, go);
	input [3:0] KEY;
	input clock;
	input [7:0] button;
	input [2:0] toBePressed;
	output go;
	
	reg [2:0] currentState, nextState;
	localparam start=3'b000, first1=3'b001, first0=3'b011, last1=3'b101; 
	
	always@(*)
	begin: state_table
		case(currentState)
		start: begin 
				//if(button == 0) nextState = first1;//once they press down move to the next state
				
				if((toBePressed==3'b111)&&(button[7] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b110)&&(button[6] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b101)&&(button[5] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b100)&&(button[4] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b011)&&(button[3] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b010)&&(button[2] == 0)) begin
						nextState = first1;
				end
					else if((toBePressed==3'b001)&&(button[1] == 0)) begin
						nextState = first1;
				end				
					else if((toBePressed==3'b000)&&(button[0] == 0)) begin
						nextState = first1;
				end
				
				
				else nextState = start;
		end
		first1: begin
				if((toBePressed==3'b111)&&(button[7] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b110)&&(button[6] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b101)&&(button[5] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b100)&&(button[4] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b011)&&(button[3] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b010)&&(button[2] == 1)) begin
						nextState = last1;
				end
					else if((toBePressed==3'b001)&&(button[1] == 1)) begin
						nextState = last1;
				end				
					else if((toBePressed==3'b000)&&(button[0] == 1)) begin
						nextState = last1;
				end
				
				else nextState =  first1;//so once they've let go move to the next state
					//this also means that they've pressed and let go of the button once
		end
		last1: begin
				nextState = start; 
		end
		default : nextState = start; 
		endcase
	end
	
	always @(posedge clock)
    begin: state_FFs			
      if(KEY[0] == 1'b0)
         currentState <= start; // Should set reset state to state A
      else
			currentState <= nextState;
   end // state_FFS
	 
	assign go = (currentState == last1);
endmodule






module NESReader (CLOCK_50, GPIO_0, KEY, button, GPIO_1); 
	input CLOCK_50; 
	input [3:0] KEY; 
	input [1:0] GPIO_0;
	output [1:0] GPIO_1; 
	output [7:0] button;
	//output [9:0] LEDR;
	topLOAD GOLD5 (CLOCK_50, GPIO_0[0], KEY, button[7], button[6], button[5], button[4], button[3], button[2], button[1], button[0], GPIO_1[0], GPIO_1[1]);
	//not too sure how gpio is gonna work here but we'll figure it out
	//gpio=data from controller
endmodule 

module topLOAD(CLOCK_50, GPIO_0, KEY, Ao, Bo, SELo, STRTo, UPo, DOWNo, LEFTo, RIGHTo, latch_out, clock_out);
	input CLOCK_50;
	input [3:0] KEY; 
	input GPIO_0; 
	
	output Ao, Bo, SELo, STRTo, UPo, DOWNo, LEFTo, RIGHTo, latch_out, clock_out; 
	
	//^^these go to all of them
	wire ltchCount, clkCount, go, go2; 
	
	wire [3:0] ldCount;//also called count cycle
	//^^from the clocks to datapath/control
	
	wire latchcounter_enable, clockcounter_enable, count7_enable, goen, goen2;
	//^^from datapath to counters
	
	wire ld_latch_count, ld_latch, ld_a, ld_clock_count, ld_count_cycle, ld_clock, ld_bit;
	//^^from control to datapath
	
	wire A, B, SEL, STRT, UP, DOWN, LEFT, RIGHT; 
	wire latch_temp, clock_temp; 
	//^^from datapath to out
	
	latch_count latchcounter (CLOCK_50, KEY[0], latchcounter_enable, ltchCount);
	latch_count clockcounter (CLOCK_50, KEY[0], clockcounter_enable, clkCount);
	count7 load_counter (CLOCK_50, KEY[0], count7_enable, clkCount, clock_temp,ldCount); 
	clock_count delayclock (CLOCK_50, KEY[0], goen, go);
	clock_count delayclock2 (CLOCK_50, KEY[0], goen2, go2);
	
	control grandmaster(CLOCK_50, KEY[0], ldCount, go, go2, ltchCount, clkCount, ld_latch_count, ld_latch, ld_a, ld_clock_count, ld_count_cycle, ld_clock, ld_bit);
	dataPath diamond1(CLOCK_50, KEY[0], ldCount, ld_latch_count, ld_latch, ld_a, ld_clock_count, ld_count_cycle, ld_clock, ld_bit, GPIO_0, latch_temp, clock_temp, A, B, SEL, STRT, UP, DOWN, LEFT, RIGHT, latchcounter_enable, clockcounter_enable, count7_enable, goen, goen2);
	
	assign Ao = A; 
	assign Bo = B; 
	assign SELo = SEL;
	assign STRTo = STRT; 
	assign UPo = UP; 
	assign DOWNo = DOWN; 
	assign LEFTo = LEFT; 
	assign RIGHTo = RIGHT; 
	assign latch_out = latch_temp; 
	assign clock_out = clock_temp; 
	
endmodule


module control (clock, reset, ldCount, go, go2, ltchCount, clkCount, ld_latch_count, ld_latch, ld_a, ld_clock_count, ld_count_cycle, ld_clock, ld_bit);
	input clock, reset;//more??????? 
	input [3:0] ldCount;
	input go, go2, ltchCount, clkCount; 
	
	output reg 	ld_latch_count, ld_clock_count, ld_latch, ld_a, ld_clock, ld_count_cycle, ld_bit; 
	
	reg [5:0] current_state, next_state;  //so states are done ezklap	
	
	localparam  S_WAIT_LATCH	= 5'd0, 
					S_SET_LATCH		= 5'd1,
					S_WAIT			= 5'd2,
					S_LOAD_A			= 5'd3,
					S_WAIT_CLK		= 5'd4,
					S_SET_CLK		= 5'd5,
					S_LOAD_BIT		= 5'd6; 
	
	always@(*)
	begin: state_table 
            case (current_state)
					S_WAIT_LATCH: begin 
						if(ltchCount == 1) next_state = S_SET_LATCH; 
						else next_state = S_WAIT_LATCH; //d
					end 
					S_SET_LATCH: begin
						if(go == 1) next_state = S_WAIT; 
						else next_state = S_SET_LATCH; //d
					end 
					S_WAIT: begin
						next_state = S_LOAD_A; //d
					end 
					S_LOAD_A: begin
						next_state = S_WAIT_CLK; //d
					end
					S_WAIT_CLK: begin
						if(ldCount < 4'b1000)begin
							if(clkCount == 1) next_state = S_SET_CLK; 
							else next_state = S_WAIT_CLK;
						end
						else next_state = S_WAIT_LATCH; 
					end 
					S_SET_CLK: begin
						if(go2 == 1) next_state = S_LOAD_BIT; 
						else next_state = S_SET_CLK; //d
					end
					S_LOAD_BIT: begin
						next_state = S_WAIT_CLK; //d
					end
					
				default: next_state = S_WAIT_LATCH;
        endcase
    end 
	
	always @(*)
	begin: enable_signals
		ld_latch_count = 1'b0; 
		ld_clock_count = 1'b0;
		ld_latch = 1'b0; 
		ld_a = 1'b0; 
		ld_latch_count = 1'b0;
		ld_clock = 1'b0; 
		ld_count_cycle = 1'b0; 
		ld_bit = 1'b0; 
		
		case (current_state)
			S_WAIT_LATCH: begin
				ld_latch_count = 1'b1; 
			end
			S_SET_LATCH: begin
				ld_latch = 1'b1;
			end
			S_LOAD_A: begin
				ld_a = 1'b1; 
			end 
			S_WAIT_CLK: begin
				ld_clock_count = 1'b1; 
				ld_count_cycle = 1'b1; 
			end 
			S_SET_CLK: begin
				ld_clock = 1'b1;
				ld_count_cycle = 1'b1; 
			end 
			S_LOAD_BIT: begin 
				ld_bit = 1'b1; 
				ld_count_cycle = 1'b1;
			end 
			
		endcase
   end
	
	always@(posedge clock)
    begin: state_FFs
        if(!reset)
            current_state <= S_WAIT_LATCH;
        else
            current_state <= next_state;
    end
	
endmodule

module dataPath (clock, reset, ldCount, ld_latch_count, ld_latch, ld_a, ld_clock_count, ld_count_cycle, ld_clock, ld_bit, GPIO_0, latch_out, clock_out, A, B, SEL, STRT, UP, DOWN, LEFT, RIGHT, latchcounter_enable, clockcounter_enable, count7_enable, goen, goen2); 
	input clock, reset, ld_latch_count, ld_latch, ld_a, ld_clock_count, ld_count_cycle, ld_clock, ld_bit; 
	input GPIO_0;
	input [3:0] ldCount; 
	
	output reg latchcounter_enable, clockcounter_enable, count7_enable, goen, goen2; 

	output reg latch_out, clock_out; 
	output reg A, B, SEL, STRT, UP, DOWN, LEFT, RIGHT; 
	
	//FOR BUTTON STATE IT HAS BEEN PRESSED IF A 0 IS SEEN
	//THE ORDER OF THE BUTTONS IN BUTTON STATE IS AS FOLLOWS:A,B,SEL,START,UP,DOWN,LEFT,RIGHT
	
	reg [7:0] button_state; 

	always@(posedge clock) begin
		if(!reset) begin
			button_state = 8'b11111111;
			latchcounter_enable <= 1'b0; 
			count7_enable <= 1'b0; 
			latch_out <= 1'b0;
			clock_out <= 1'b0; 
			clockcounter_enable <= 1'b0; 
			goen <= 1'b0; 
			goen2 <= 1'b0;
		end
		else begin 
				if(ld_latch_count) latchcounter_enable <= 1'b1; 
				else begin
					latchcounter_enable <= 1'b0;
					count7_enable <= 1'b0; 
				end
					
				if(ld_latch) begin
					latch_out <= 1'b1;
					goen <= 1'b1; 
				end
				else begin 
					latch_out <= 1'b0;
					goen <= 1'b0; 
				end 
					
				if(ld_a) button_state[7] <= GPIO_0; //might need to change this
					
				if(ld_clock_count) clockcounter_enable <= 1'b1; 
				else clockcounter_enable <= 1'b0; 
				
				if(ld_count_cycle) count7_enable <= 1'b1;
				else count7_enable <= 1'b0; 
					
				if(ld_clock) begin 
					clock_out <= 1'b1;
					goen2 <= 1'b1; 
				end 
				else begin 
					clock_out <= 1'b0;
					goen2 <= 1'b0; 
				end 
					
				if(ld_bit) begin
					if(ldCount == 4'b0001) button_state[6] <= GPIO_0;
					if(ldCount == 4'b0010) button_state[5] <= GPIO_0;
					if(ldCount == 4'b0011) button_state[4] <= GPIO_0;
					if(ldCount == 4'b0100) button_state[3] <= GPIO_0;
					if(ldCount == 4'b0101) button_state[2] <= GPIO_0;
					if(ldCount == 4'b0110) button_state[1] <= GPIO_0;
					if(ldCount == 4'b0111) button_state[0] <= GPIO_0;						
				end //might need to change this 
				
			end
		end
	
	//always part 2??!?!??!?!??!?!?!?!?!?!??!??!?!?!?!?!?!?!??!?!?!??!?!?!?!?
	always@(posedge clock)begin
		//update all the buttons pressed with the states here *dab*
		if(!reset) begin
			A <= 1'b1; 
			B <= 1'b1;
			SEL <= 1'b1;
			STRT <= 1'b1;
			UP <= 1'b1;
			DOWN <= 1'b1;
			LEFT <= 1'b1;
			RIGHT <= 1'b1;
		end
		else begin
			A <= button_state[7];
			B <= button_state[6];
			SEL <= button_state[5];
			STRT <= button_state[4];
			UP <= button_state[3];
			DOWN <= button_state[2];
			LEFT <= button_state[1];
			RIGHT <= button_state[0];
		end
	end
	
endmodule  
 
module latch_count (clock, reset, enable, q);
	input clock, reset, enable; 
	output reg q;
	
	reg [9:0] value; 
	
	always@(negedge reset, posedge clock)
	begin 
		if(reset == 1'b0) begin
			value <= 10'b1001010111; 
			q <= 1'b0; 
		end 
		else if(enable == 1'b0) begin
			value <= 10'b1001010111;
			q <= 1'b0; 
		end 
		else if (value == 10'b0000000000) begin
			value <= 10'b1001010111;
			q <= 1'b1; 
		end 
		else if (enable == 1'b1) 
			value <= value - 1; 
	end 
endmodule

module clock_count (clock, reset, enable, q);
	input clock, reset, enable; 
	output reg q;
	
	reg [8:0] value; 
	
	always@(negedge reset, posedge clock)
	begin 
		if(reset == 1'b0) begin
			value <= 9'b100101011; 
			q <= 1'b0; 
		end 
		else if(enable == 1'b0) begin
			value <= 9'b100101011;
			q <= 1'b0; 
		end 
		else if (value == 9'b000000000) begin
			value <= 9'b100101011;
			q <= 1'b1; 
		end 
		else if (enable == 1'b1) 
			value <= value - 1; 
	end 
endmodule

module count7 (clock, reset, enable1, enable2, enable3, q);
	input clock, reset, enable1, enable2, enable3; 
	output reg [3:0] q; 
	
	always@(negedge reset, posedge clock)
	begin 
		if(reset == 1'b0) 
			q <= 4'b000; 
		else if (enable1 == 1'b0)
			q <= 4'b0000;   
		else if (q == 4'b1001)
			q <= 4'b0000; 
		else if (enable1 == 1)
			if(enable2 == 1) 
				if(enable3 == 1) q <= q + 1; 
	end 
endmodule    
//controller end

