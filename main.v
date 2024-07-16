// Part 2 skeleton

module fill
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		KEY,							// On Board Keys
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,					//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,							//	VGA Blue[9:0]
		PS2_CLK, 
		PS2_DAT,
		HEX0,
		HEX1,
		HEX2,
		HEX3,
		HEX4,
		HEX5
	);

	input		CLOCK_50;				//	50 MHz
	input	[3:0]	KEY;					
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
	
	
	input PS2_DAT;
	input PS2_CLK;
	
	output [6:0] HEX0;
	output [6:0] HEX1;
	output [6:0] HEX2;
	output [6:0] HEX3;
	output [6:0] HEX4;
	output [6:0] HEX5;
	
	wire resetn;
	wire drawKey;
	wire OurReset;
	wire jump;
	
	assign resetn = KEY[0];
	assign drawKey = ~KEY[1];
	assign OurReset = ~KEY[2];
	
	combined u2(PS2_CLK, PS2_DAT, jump, signal);
	
	
	ScoreTracker u3(
    CLOCK_50,   
		HEX0,
		HEX1,
		HEX2,
		HEX3,
		HEX4,
		HEX5,
		drawKey
);
	
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	
	// keyboard
	wire keyboard_reset;
	wire [7:0] ikeyboard_data;
	wire ikeyboard_data_en;



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
		defparam VGA.BACKGROUND_IMAGE = "image.colour.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.

	finalDraw u1(CLOCK_50,OurReset,drawKey,jump,x,y,colour,writeEn);
	
	
endmodule

module finalDraw(clk,Reset,drawKey,jump,x,y,colour,plot);

localparam	
				S_Reset = 5'd0,
				S_Start = 5'd1,
				S_Start_wait = 5'd2,
				S_Play = 5'd3,
				S_GameOver = 5'd4,
				S_GameOverWait = 5'd5;

input clk,Reset,drawKey,jump;

output reg [7:0] x;
output reg [6:0] y;

output reg[3:0] colour;
output reg plot;

wire clockHalf;
wire [7:0] xDinasour,xObstacle;
wire [6:0] yDinasour,yObstacle;	
wire [2:0] colourDinasour, colourObstacle;
wire plotDinasour,plotObstacle;

wire [2:0]colorImageStart;

wire gameOver;

reg[4:0] current_state,next_state;

	
	Collision u1(clk,Reset,xDinasour,yDinasour,xObstacle,yObstacle,gameOver);
	RateDividerHalf u2(clk, clockHalf,Reset);
	DrawDinasour u3(clk,Reset, xDinasour,yDinasour,8'd10,7'd90,plotDinasour,drawKey,colourDinasour,jump);
	DrawObstacle u4(clk,Reset,xObstacle,yObstacle,8'd154,7'd90,plotObstacle,drawKey,colourObstacle);
	
	always@(posedge clk)
	begin
		if(Reset) current_state = S_Reset;
		else current_state= next_state;
	end
	
	always@(*)
	begin
		case(current_state)
			S_Reset: next_state = S_Start;
			S_Start: next_state = drawKey? S_Start_wait: S_Start;
			S_Start_wait: next_state= drawKey ? S_Start_wait: S_Play;
			S_Play: next_state = gameOver? S_GameOver:S_Play;
			S_GameOver : next_state = Reset? S_GameOverWait: S_GameOver;
			S_GameOverWait: next_state = Reset? S_GameOverWait: S_Reset;
		endcase
	end
	
	always@(posedge clk)
	case(current_state)
	
	S_Reset:
	begin
		x<= 8'd154;
		y<=7'd90;
	end
	
	S_Play:
		if(clockHalf)
		begin
			x<= xDinasour;
			y<= yDinasour;
			colour<= colourDinasour;
			plot <= plotDinasour;
		end
		
		else
		begin
			x<= xObstacle;
			y<= yObstacle;
			colour<= colourObstacle;
			plot <= plotObstacle;
		end
		
		S_GameOver:
			begin
				x<=1;
				y<=1;
			end
		

endcase
endmodule



module DrawDinasour (clk,Reset,x,y,initialX,initialY,load,Start,color,jump);

wire DoneDraw;
wire gameRate;

output reg[2:0] color;

reg allowDraw;
output [7:0] x;
output [6:0] y;

input clk,Reset,jump,Start;
input [7:0]initialX;
input [6:0]initialY;
output load;

reg [6:0] current_y;

reg [5:0]current_state,next_state;



localparam 
				S_Draw = 5'd1,
				S_Jump = 5'd2,
				S_Delete = 5'd3,
				S_Start = 5'd4,
				S_Start_wait = 5'd5,
				S_Reset =5'd6,
				S_Done = 5'd7,
				S_DeleteOff = 5'd8,
				S_DrawOff = 5'd9,
				S_Fall = 5'd10,
				S_Ground = 5'd11,
				S_JumpDelete = 5'd15,
				S_FallDelete = 5'd16,
				S_GroundDelete = 5'd17,
				S_GroundDeleteOff = 5'd18,
				S_FallDeleteOff = 5'd19,
				S_JumpDeleteOff = 5'd20,
				S_JumpDrawOff = 5'd21,
				S_FallDrawOff = 5'd22,
				S_GroundDrawOff = 5'd23,
				S_JumpCount = 5'd24,
				S_FallCount = 5'd25,
				S_GroundCount = 5'd26;



	RateDivider u2(clk, gameRate, Reset);

	FSMDraw u1(clk,Reset,x,y,initialX,current_y,load,allowDraw,DoneDraw,gameRate);
	
always@(posedge gameRate)
begin
	if(Reset) current_state <= S_Reset;
	else current_state <= next_state;

end



always@(*)
	case(current_state)
		S_Reset: next_state = S_Start;
		
		S_Start: next_state = Start ? S_Start_wait: S_Start;
		S_Start_wait: next_state = Start? S_Start_wait: S_Draw;
		S_Draw:  next_state = DoneDraw ? S_DrawOff : S_Draw;
		S_DrawOff: next_state = S_Delete ;
		S_Delete: next_state = DoneDraw ? S_DeleteOff : S_Delete;
		
		S_DeleteOff: next_state = S_Ground;
		
		S_Ground : next_state = DoneDraw ? S_GroundDrawOff : S_Ground;
		S_GroundDrawOff : next_state = S_GroundDelete;
		S_GroundDelete: next_state = DoneDraw ? S_GroundDeleteOff : S_GroundDelete;
		S_GroundDeleteOff: next_state = jump ? S_JumpCount:S_GroundCount;
		
		S_Jump : next_state = DoneDraw ? S_JumpDrawOff : S_Jump;
		S_JumpDrawOff : next_state = S_JumpDelete;
		S_JumpDelete: next_state = DoneDraw ? S_JumpDeleteOff : S_JumpDelete;
		S_JumpDeleteOff: begin
			if(jump && current_y>=7'd75)next_state = S_JumpCount;
		else if(current_y < 7'd90)next_state = S_FallCount;
		else next_state = S_GroundCount;
		end
		
		
		S_Fall : next_state = DoneDraw ? S_FallDrawOff : S_Fall;
		S_FallDrawOff : next_state = S_FallDelete;
		S_FallDelete: next_state = DoneDraw ? S_FallDeleteOff : S_FallDelete;
		S_FallDeleteOff: next_state = (current_y<7'd90) ? S_FallCount : S_GroundCount;
		
		S_FallCount: next_state = S_Fall;
		S_JumpCount: next_state = S_Jump;
		S_GroundCount: next_state = S_Ground;
		
		S_Done: next_state = S_Draw;
		
		default: next_state = S_Reset;
	endcase
	
	always @(posedge gameRate)
		case(current_state)
			S_Reset:begin 
				allowDraw <= 1'b0;
				current_y <= initialY;
				
			end
			
			S_JumpCount: begin
			current_y <= current_y - 1;
			end
			
			S_FallCount: begin
			current_y <= current_y +1;
			end
			
			S_GroundCount : begin
			current_y <= 7'd90;
			end
			
			S_GroundDrawOff :allowDraw <= 1'b0;
			S_FallDrawOff :allowDraw <= 1'b0;
			S_JumpDrawOff :allowDraw <= 1'b0;
			
			S_JumpDelete:begin
				allowDraw <= 1'b1;
				color <= 3'b010;
			end
			
			S_FallDelete:begin
				allowDraw <= 1'b1;
				color <= 3'b010;
			end
			
			S_GroundDelete:begin
				allowDraw <= 1'b1;
				color <= 3'b010;
			end
			
			S_JumpDeleteOff: allowDraw <= 1'b0;	
			S_GroundDeleteOff: allowDraw <= 1'b0;
			S_FallDeleteOff: allowDraw <= 1'b0;
			
			S_Jump:begin
				color <= 3'b101;
				allowDraw <= 1'b1;
			end
			
			S_Fall:begin
				color <= 3'b101;
				allowDraw <= 1'b1;
			end
			
			S_Ground:begin
				color <= 3'b101;
				allowDraw <= 1'b1;
			end
			
			S_Delete:begin
				allowDraw <= 1'b1;
				color <= 3'b010;
			end
			S_DeleteOff: allowDraw <= 1'b0;
			S_DrawOff :allowDraw <= 1'b0;
			S_Draw:begin
				color <= 3'b101;
				allowDraw <= 1'b1;
			end
			S_Done:begin
				allowDraw <= 1'b0;
				current_y <= 8'd154;
			end
			S_Start:begin
				allowDraw <= 1'b1;
			end
			
		endcase

endmodule

module DrawObstacle(clk,Reset,x,y,initialX,initialY,load,Start,color);


input Start;
wire DoneDraw;
wire gameRate;

output reg[2:0] color;

reg allowDraw;
output [7:0] x;
output [6:0] y;

input clk,Reset;
input [7:0]initialX;
input [6:0]initialY;
output load;

reg [7:0] current_x;

reg [5:0]current_state,next_state;



localparam 
				S_Draw = 5'd1,
				S_Count = 5'd2,
				S_Delete = 5'd3,
				S_Start = 5'd4,
				S_Start_wait = 5'd5,
				S_Reset =5'd6,
				S_Done = 5'd7,
				S_DeleteOff = 5'd8,
				S_DrawOff = 5'd9;



	RateDivider u2(clk, gameRate, Reset);

	FSMDraw u1(clk,Reset,x,y,current_x,initialY,load,allowDraw,DoneDraw,gameRate);

always@(posedge gameRate)
begin
	if(Reset) current_state <= S_Reset;
	else current_state <= next_state;

end



always@(*)
	case(current_state)
		S_Reset: next_state = S_Start;
		
		S_Start: next_state = Start ? S_Start_wait: S_Start;
		S_Start_wait: next_state = Start? S_Start_wait: S_Draw;
		S_Draw:  next_state = DoneDraw ? S_DrawOff : S_Draw;
		S_DrawOff: next_state = S_Delete ;
		S_Delete: next_state = DoneDraw ? S_DeleteOff : S_Delete;
		
		S_DeleteOff:
		begin 
		
		if(current_x == 8'd0)
		next_state = S_Done;
		
		else
		next_state = S_Count;
		
		end
		S_Count: next_state = S_Draw;
		S_Done: next_state = S_Draw;
		
		default: next_state = S_Reset;
	endcase
	
	always @(posedge gameRate)
		case(current_state)
			S_Reset:begin 
				allowDraw <= 1'b0;
				current_x <= initialX;
			end
			S_Count:begin
			current_x <= current_x - 1;
			end
			
			S_Delete:begin
				allowDraw <= 1'b1;
				color <= 3'b010;
			end
			S_DeleteOff: allowDraw <= 1'b0;
			S_DrawOff :allowDraw <= 1'b0;
			S_Draw:begin
				color <= 3'b001;
				allowDraw <= 1'b1;
			end
			S_Done:begin
				allowDraw <= 1'b0;
				current_x <= 8'd154;
			end
			S_Start:begin
				allowDraw <= 1'b1;
			end
			
		endcase
		

endmodule



module FSMDraw(clk,Reset,x,y,initialX,initialY,load,draw,DoneDraw,gameRate);

input gameRate;
reg[5:0] current_state, next_state;
output reg[7:0] x;
output reg[6:0] y;

output reg DoneDraw;
reg [4:0] count;

input draw,Reset,clk;
input [7:0]initialX;
input [6:0]initialY;
output reg load;



localparam	

				S_DrawPixel = 5'd0,
				S_Loadoff = 5'd1,
				S_Count = 5'd2,
				S_reset = 5'd3,
				S_done = 5'd4,
				S_wait_signal= 5'd5;
				
				always@(*)
				begin: state_table
					case(current_state)
					S_wait_signal : next_state = draw ? S_DrawPixel : S_wait_signal;
					S_reset: next_state = S_wait_signal;
					S_DrawPixel: begin
						if(count[4] == 1'b1) next_state = S_done;
						else next_state = S_Loadoff;
					end
					S_Loadoff: next_state = S_Count;
					S_Count: next_state = S_DrawPixel;
					S_done : next_state = gameRate ? S_reset : S_done;
					default: next_state = S_reset;
					
					endcase
				end

				always@(posedge clk)
				
				begin
					if(Reset) current_state <= S_reset;
					else current_state <= next_state;
				end
				
				always@(posedge clk)
				begin
				
				load <= 1'b0;
				
				case(current_state)
					S_reset: begin
						x <= initialX;
						y <= initialY;
						load <= 1'b0;
						count <= 4'b0000;
						DoneDraw = 1'b0;
					end
					S_DrawPixel: begin
						load <= 1'b1;
					end
					S_Loadoff: load <= 0;
					S_Count: begin
						count <= count + 1;
						x <= initialX+count[1:0];
						y <= initialY+count[3:2];
					end
					S_done : begin
						load <= 0;
						DoneDraw <= 1'b1;
					end
				endcase
				
				end
				
endmodule

module RateDivider#(parameter CLOCK_FRECUENCY = 50000000)(input ClockIn, output reg clockOut, input Reset);


reg [$clog2(CLOCK_FRECUENCY*4-1):0] MaxValue;

        always@(posedge ClockIn)
        begin
        if(Reset == 1)
        begin
					clockOut <= 0;
                MaxValue<=(CLOCK_FRECUENCY-1)/1200;

        end
		  
		  
        else
		  
        begin  
		  
        if(MaxValue == 0)
        begin
            MaxValue<=(CLOCK_FRECUENCY-1)/1200;
				clockOut <= !clockOut;
        end
		  
        else
        begin
            MaxValue <= MaxValue -1;
        end
		  
        end
		  
		  end

endmodule

module RateDividerHalf#(parameter CLOCK_FRECUENCY = 50000000)(input ClockIn, output reg clockOut, input Reset);


reg [$clog2(CLOCK_FRECUENCY*4-1):0] MaxValue;

        always@(posedge ClockIn)
        begin
        if(Reset == 1)
        begin
					clockOut <= 0;
                MaxValue<=2;

        end
		  
		  
        else
		  
        begin  
		  
        if(MaxValue == 0)
        begin
            MaxValue<=2;
				clockOut <= !clockOut;
        end
		  
        else
        begin
            MaxValue <= MaxValue -1;
        end
		  
        end
		  
		  end

endmodule

module Collision (
  input clk,          
  input reset,         
  input[7:0] dino_x,        
  input [6:0]dino_y,         
  input [7:0]obs_x,				
	input [6:0]obs_y,				
  output reg gameOver        
);

  // Collision parameters
  reg [9:0] collision_range;

  // Initialize
  always @(posedge clk)
  begin
    if (reset)
    begin
      collision_range <= 10'd1; // You can adjust the collision range
      gameOver <= 1'b0;
    end
    else
    begin
      // Check for collision
      if (((obs_x - dino_x <= collision_range) || (  dino_x - obs_x <= collision_range)) &&
          (obs_y-dino_y <= collision_range))
        gameOver <= 1'b1;
    end
  end

endmodule


module combined(PS2_CLK, PS2_DAT, jump, signal,LEDR);
    input PS2_CLK, PS2_DAT;
    output reg jump = 1'b0;
	 output reg [2:0] signal;
	 output [1:0]LEDR;
    reg [3:0] counter = 4'b0;
	 reg [7:0] data = 8'b0;
    reg done = 1'b0, pulse_down = 1'b0;
	 assign LEDR[0] = jump;
    always @(negedge PS2_CLK) begin
        case(counter)
            4'd0:   ;
            4'd1:   data[0] <= PS2_DAT;
            4'd2:   data[1] <= PS2_DAT;
            4'd3:   data[2] <= PS2_DAT;
            4'd4:   data[3] <= PS2_DAT;
            4'd5:   data[4] <= PS2_DAT;
            4'd6:   data[5] <= PS2_DAT;
            4'd7:   data[6] <= PS2_DAT;
            4'd8:   data[7] <= PS2_DAT;
            4'd9:   done <= 1'b1;
            4'd10:  done <= 1'b0;
            default: counter <= 0;
        endcase
        if(counter == 10) counter <= 0;
        else counter <= counter+1;
    end
    always @(posedge done) begin
        if(data == 8'hF0) begin
            pulse_down <= 1;
				
        end
        else begin
            if(data == 8'h29) begin
				jump <= !pulse_down;
				if(pulse_down != 1'b1) signal <= 3'b111; 
				end
            else signal <= 3'b000;
				pulse_down <= 0;
				
        end
    end
endmodule

module ScoreTracker (
  input  CLOCK_50,   // Clock input
  // Reset input 
  //output reg [3:0] score,   // 32-bit score output
output [6:0] HEX0,
output[6:0] HEX1,
output [6:0] HEX2,
output[6:0] HEX3,
output [6:0] HEX4,
output[6:0] HEX5,

input Reset
);

initial begin
	 counter = 0;
	 score = 0;
end


reg [23:0] score;
reg [15:0] counter;

hex_decoder h1(.c(score[3:0]),.display(HEX0));
hex_decoder h2(.c(score[7:4]),.display(HEX1));
hex_decoder h3(.c(score[11:8]),.display(HEX2));
hex_decoder h4(.c(score[15:12]),.display(HEX3));
hex_decoder h5(.c(score[19:16]),.display(HEX4));
hex_decoder h6(.c(score[23:20]),.display(HEX5));

  
  // Always block triggered on positive edge of the clock
  always @(posedge CLOCK_50) begin
		if (Reset) begin
      // Reset the counter and score when reset is asserted
      counter <= 0;
      score <= 0;
    end
      else begin
		
		counter <= counter + 16'd1;
      // When the counter reaches the desired frequency (1 Hz), increment the score
      if (counter == 16'd250000000) begin
        score <= score + 23'd1;
        counter <= 0;  // Reset the counter
      end
		end
  end

endmodule

module hex_decoder(c,display);

input [3:0] c;
output [6:0] display;

assign display[0] = ~((c[3]|c[2]|c[1]|~c[0])&(c[3]|~c[2]|c[1]|c[0])&(~c[3]|c[2]|~c[1]|~c[0])&(~c[3]|~c[2]|c[1]|~c[0]));

assign display[1] = ~((c[3]|~c[2]|c[1]|~c[0])&(c[3]|~c[2]|~c[1]|c[0])&(~c[3]|c[2]|~c[1]|~c[0])&(~c[3]|~c[2]|c[1]|c[0])&(~c[3]|~c[2]|~c[1]|c[0])&(~c[3]|~c[2]|~c[1]|~c[0]));

assign display[2] =  ~((c[3]|c[2]|~c[1]|c[0])&(~c[3]|~c[2]|c[1]|c[0])&(~c[3]|~c[2]|~c[1]|c[0])&(~c[3]|~c[2]|~c[1]|~c[0]));

assign display[3] = ~((c[3]|c[2]|c[1]|~c[0])&(c[3]|~c[2]|c[1]|c[0])&(c[3]|~c[2]|~c[1]|~c[0])&(~c[3]|c[2]|c[1]|~c[0])&(~c[3]|c[2]|~c[1]|c[0])&(~c[3]|~c[2]|~c[1]|~c[0]));

assign display[4] =  ~((~c[3]&~c[2]&~c[1]&~c[0]) | (~c[3]&~c[2]&c[1]&~c[0])|(~c[3]&c[2]&c[1]&~c[0])|(c[3]&~c[2]&~c[1]&~c[0])|(c[3]&~c[2]&c[1]&~c[0])|(c[3]&~c[2]&c[1]&c[0])|(c[3]&c[2]&~c[1]&~c[0])|(c[3]&c[2]&~c[1]&c[0])|(c[3]&c[2]&c[1]&~c[0])|(c[3]&c[2]&c[1]&c[0]));

assign display[5] = ~((c[3]|c[2]|c[1]|~c[0])&(c[3]|c[2]|~c[1]|c[0])&(c[3]|c[2]|~c[1]|~c[0])&(c[3]|~c[2]|~c[1]|~c[0])&(~c[3]|~c[2]|c[1]|~c[0]));

assign display[6] =  ~((c[3]|c[2]|c[1]|c[0])&(c[3]|c[2]|c[1]|~c[0])&(c[3]|~c[2]|~c[1]|~c[0])&(~c[3]|~c[2]|c[1]|c[0]));

endmodule
