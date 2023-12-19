// Neil Chulani
// nchulani@g.hmc.edu
// 10/25/23

// This file contains code to implement AES-128 for Microps Lab 07

/////////////////////////////////////////////
// aes
//   Top level module with SPI interface and SPI core
/////////////////////////////////////////////

module aes(input  logic clk,
           input  logic sck, 
           input  logic sdi,
           output logic sdo,
           input  logic load,
           output logic done);
                    
    logic [127:0] key, plaintext, cyphertext;
            
    aes_spi spi(sck, sdi, sdo, done, key, plaintext, cyphertext);   
    aes_core core(clk, load, key, plaintext, done, cyphertext);
endmodule

/////////////////////////////////////////////
// aes_spi
//   SPI interface.  Shifts in key and plaintext
//   Captures ciphertext when done, then shifts it out
//   Tricky cases to properly change sdo on negedge clk
/////////////////////////////////////////////

module aes_spi(input  logic sck, 
               input  logic sdi,
               output logic sdo,
               input  logic done,
               output logic [127:0] key, plaintext,
               input  logic [127:0] cyphertext);

    logic         sdodelayed, wasdone;
    logic [127:0] cyphertextcaptured;
               
    // assert load
    // apply 256 sclks to shift in key and plaintext, starting with plaintext[127]
    // then deassert load, wait until done
    // then apply 128 sclks to shift out cyphertext, starting with cyphertext[127]
    // SPI mode is equivalent to cpol = 0, cpha = 0 since data is sampled on first edge and the first
    // edge is a rising edge (clock going from low in the idle state to high).
    always_ff @(posedge sck)
        if (!wasdone)  {cyphertextcaptured, plaintext, key} = {cyphertext, plaintext[126:0], key, sdi};
        else           {cyphertextcaptured, plaintext, key} = {cyphertextcaptured[126:0], plaintext, key, sdi}; 
    
    // sdo should change on the negative edge of sck
    always_ff @(negedge sck) begin
        wasdone = done;
        sdodelayed = cyphertextcaptured[126];
    end
    
    // when done is first asserted, shift out msb before clock edge
    assign sdo = (done & !wasdone) ? cyphertext[127] : sdodelayed;
endmodule

/////////////////////////////////////////////
// aes_core
//   top level AES encryption module
//   when load is asserted, takes the current key and plaintext
//   generates cyphertext and asserts done when complete 11 cycles later
// 
//   See FIPS-197 with Nk = 4, Nb = 4, Nr = 10
//
//   The key and message are 128-bit values packed into an array of 16 bytes as
//   shown below
//        [127:120] [95:88] [63:56] [31:24]     S0,0    S0,1    S0,2    S0,3
//        [119:112] [87:80] [55:48] [23:16]     S1,0    S1,1    S1,2    S1,3
//        [111:104] [79:72] [47:40] [15:8]      S2,0    S2,1    S2,2    S2,3
//        [103:96]  [71:64] [39:32] [7:0]       S3,0    S3,1    S3,2    S3,3
//
//   Equivalently, the values are packed into four words as given
//        [127:96]  [95:64] [63:32] [31:0]      w[0]    w[1]    w[2]    w[3]
/////////////////////////////////////////////

// core module to run AES
module aes_core(input  logic         clk,
                input  logic         load,
                input  logic [127:0] key, 
                input  logic [127:0] plaintext, 
                output logic         done, 
                output logic [127:0] cyphertext);

    // TODO: Your code goes here
	logic fsm_done;
	logic [127:0] curKey;
	logic [127:0] curPlaintext;
	
	logic [2:0] state, nextstate;
	
	parameter S0 = 3'b000;
	parameter S1 = 3'b001;
	parameter S2 = 3'b010;
	
	always_ff @(posedge clk) state <= nextstate;
	
	always_comb
		case (state)
			S0: if (load) nextstate <= S1;
				else nextstate <= S0;
			S1: if (fsm_done) nextstate <= S2;
				else nextstate <= S1;
			S2: nextstate <= S0;
			default: nextstate <= S0;
		endcase
		
	AES_FSM aes_main(curKey, curPlaintext, clk, fsm_done, cyphertext);
	
	always_ff @(posedge clk)
		case (state)
			S1: begin
				done <= 0;
				curKey <= key;
				curPlaintext <= plaintext;
			end
			S2: done <= 1;
		endcase
    
endmodule

/////////////////////////////////////////////
// sbox
//   Infamous AES byte substitutions with magic numbers
//   Combinational version which is mapped to LUTs (logic cells)
//   Section 5.1.1, Figure 7
/////////////////////////////////////////////

module sbox(input  logic [7:0] a,
            output logic [7:0] y);
            
  // sbox implemented as a ROM
  // This module is combinational and will be inferred using LUTs (logic cells)
  logic [7:0] sbox[0:255];

  initial   $readmemh("sbox.txt", sbox);
  assign y = sbox[a];
endmodule

/////////////////////////////////////////////
// sbox
//   Infamous AES byte substitutions with magic numbers
//   Synchronous version which is mapped to embedded block RAMs (EBR)
//   Section 5.1.1, Figure 7
/////////////////////////////////////////////
module sbox_sync(
	input		logic [7:0] a,
	input	 	logic 			clk,
	output 	logic [7:0] y);
            
  // sbox implemented as a ROM
  // This module is synchronous and will be inferred using BRAMs (Block RAMs)
  logic [7:0] sbox [0:255];

  initial   $readmemh("sbox.txt", sbox);
	
	// Synchronous version
	always_ff @(posedge clk) begin
		y <= sbox[a];
	end
endmodule

/////////////////////////////////////////////
// mixcolumns
//   Even funkier action on columns
//   Section 5.1.3, Figure 9
//   Same operation performed on each of four columns
/////////////////////////////////////////////

module mixcolumns(input  logic [127:0] a,
                  output logic [127:0] y);

  mixcolumn mc0(a[127:96], y[127:96]);
  mixcolumn mc1(a[95:64],  y[95:64]);
  mixcolumn mc2(a[63:32],  y[63:32]);
  mixcolumn mc3(a[31:0],   y[31:0]);
endmodule

/////////////////////////////////////////////
// mixcolumn
//   Perform Galois field operations on bytes in a column
//   See EQ(4) from E. Ahmed et al, Lightweight Mix Columns Implementation for AES, AIC09
//   for this hardware implementation
/////////////////////////////////////////////

module mixcolumn(input  logic [31:0] a,
                 output logic [31:0] y);
                      
        logic [7:0] a0, a1, a2, a3, y0, y1, y2, y3, t0, t1, t2, t3, tmp;
        
        assign {a0, a1, a2, a3} = a;
        assign tmp = a0 ^ a1 ^ a2 ^ a3;
    
        galoismult gm0(a0^a1, t0);
        galoismult gm1(a1^a2, t1);
        galoismult gm2(a2^a3, t2);
        galoismult gm3(a3^a0, t3);
        
        assign y0 = a0 ^ tmp ^ t0;
        assign y1 = a1 ^ tmp ^ t1;
        assign y2 = a2 ^ tmp ^ t2;
        assign y3 = a3 ^ tmp ^ t3;
        assign y = {y0, y1, y2, y3};    
endmodule

/////////////////////////////////////////////
// galoismult
//   Multiply by x in GF(2^8) is a left shift
//   followed by an XOR if the result overflows
//   Uses irreducible polynomial x^8+x^4+x^3+x+1 = 00011011
/////////////////////////////////////////////

module galoismult(input  logic [7:0] a,
                  output logic [7:0] y);

    logic [7:0] ashift;
    
    assign ashift = {a[6:0], 1'b0};
    assign y = a[7] ? (ashift ^ 8'b00011011) : ashift;
endmodule

// module to perform shift rows operation
module shiftrows (
	input logic [127:0] instate,
	output logic [127:0] outstate
);
	// Row 0 shifted by 0
	assign outstate [127:120] = instate [127:120];
	assign outstate [95:88] = instate [95:88];
	assign outstate [63:56] = instate [63:56];
	assign outstate [31:24] = instate [31:24];
	
	// Row 1 shifted by 1 
	assign outstate [119:112] = instate [87:80];
	assign outstate [87:80] = instate [55:48];
	assign outstate [55:48] = instate [23:16];
	assign outstate [23:16] = instate [119:112];
	
	// Row 2 shifted by 2
	assign outstate [111:104] = instate [47:40];
	assign outstate [79:72] = instate [15:8];
	assign outstate [47:40] = instate [111:104];
	assign outstate [15:8] = instate [79:72];
	
	// Row 3 shifted by 3
	assign outstate [103:96] = instate [7:0];
	assign outstate [71:64] = instate [103:96];
	assign outstate [39:32] = instate [71:64];
	assign outstate [7:0] = instate [39:32];
endmodule

// THIS TESTBENCH WORKS PROPERLY!
module shiftrows_tb (input logic [127:0] instate,
				  output logic [127:0] outstate);
	shiftrows dut(instate, outstate);
endmodule

// module to perform subbytes (synchronous using block ram)
module subbytes(
	input logic [127:0] instate,
	input logic clk,
	output logic [127:0] outstate
);

	logic [7:0] sbox_inputs [15:0];
	logic [7:0] sbox_outputs [15:0];
	
	assign sbox_inputs[0] = instate[7:0];
	assign sbox_inputs[1] = instate[15:8];
	assign sbox_inputs[2] = instate[23:16];
	assign sbox_inputs[3] = instate[31:24];
	assign sbox_inputs[4] = instate[39:32];
	assign sbox_inputs[5] = instate[47:40];
	assign sbox_inputs[6] = instate[55:48];
	assign sbox_inputs[7] = instate[63:56];
	assign sbox_inputs[8] = instate[71:64];
	assign sbox_inputs[9] = instate[79:72];
	assign sbox_inputs[10] = instate[87:80];
	assign sbox_inputs[11] = instate[95:88];
	assign sbox_inputs[12] = instate[103:96];
	assign sbox_inputs[13] = instate[111:104];
	assign sbox_inputs[14] = instate[119:112];
	assign sbox_inputs[15] = instate[127:120];
	
	sbox_sync s0(sbox_inputs[0], clk, sbox_outputs[0]);
	sbox_sync s1(sbox_inputs[1], clk, sbox_outputs[1]);
	sbox_sync s2(sbox_inputs[2], clk, sbox_outputs[2]);
	sbox_sync s3(sbox_inputs[3], clk, sbox_outputs[3]);
	sbox_sync s4(sbox_inputs[4], clk, sbox_outputs[4]);
	sbox_sync s5(sbox_inputs[5], clk, sbox_outputs[5]);
	sbox_sync s6(sbox_inputs[6], clk, sbox_outputs[6]);
	sbox_sync s7(sbox_inputs[7], clk, sbox_outputs[7]);
	sbox_sync s8(sbox_inputs[8], clk, sbox_outputs[8]);
	sbox_sync s9(sbox_inputs[9], clk, sbox_outputs[9]);
	sbox_sync s10(sbox_inputs[10], clk, sbox_outputs[10]);
	sbox_sync s11(sbox_inputs[11], clk, sbox_outputs[11]);
	sbox_sync s12(sbox_inputs[12], clk, sbox_outputs[12]);
	sbox_sync s13(sbox_inputs[13], clk, sbox_outputs[13]);
	sbox_sync s14(sbox_inputs[14], clk, sbox_outputs[14]);
	sbox_sync s15(sbox_inputs[15], clk, sbox_outputs[15]);
	
	always_ff @(posedge clk) begin
		outstate = {sbox_outputs[15], sbox_outputs[14], sbox_outputs[13], sbox_outputs[12],
                sbox_outputs[11], sbox_outputs[10], sbox_outputs[9], sbox_outputs[8],
                sbox_outputs[7], sbox_outputs[6], sbox_outputs[5], sbox_outputs[4],
                sbox_outputs[3], sbox_outputs[2], sbox_outputs[1], sbox_outputs[0]};
	end
endmodule

module subbytes_tb (input logic [127:0] instate, output logic [127:0] outstate);
	logic int_osc;
	HSOSC #(.CLKHF_DIV(2'b01)) 
         hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc));
		 
	subbytes dut(instate, int_osc, outstate);
endmodule

// module to perform add round key operation
module addroundkey (
	//input logic [31:0] w0,
	//input logic [31:0] w1,
	//input logic [31:0] w2,
	//input logic [31:0] w3,
	input logic [127:0] instate,
	input logic [127:0] roundkey,
	output logic [127:0] outstate
);
	
	//outstate[127:96] = instate[127:96] ^ w0;
	//outstate[95:64] = instate[95:64] ^ w1;
	//outstate[63:32] = instate[63:32] ^ w2;
	//outstate[31:0] = instate[31:0] ^ w3;
	assign outstate[127:0] = roundkey[127:0] ^ instate[127:0];
endmodule

module addroundkey_tb (input logic [127:0] roundkey, input logic [127:0] instate, output logic [127:0] outstate);
	addroundkey dut(roundkey, instate, outstate);
endmodule

// module to perform rot word operation
module rotword (
	input logic [7:0] inword [0:3],
	output logic [7:0] outword [0:3]
);
	assign outword[0] = inword[1];
	assign outword[1] = inword[2];
	assign outword[2] = inword[3];
	assign outword[3] = inword[0];
endmodule

module rotword_tb (input logic [7:0] inword [0:3], output logic [7:0] outword [0:3]);
	rotword dut(inword, outword);
endmodule

// module to perform sub word operation (synchronous using block ram)
module subword (
	input logic [7:0] inword [0:3],
	input logic clk,
	output logic [7:0] outword [0:3]
);
	logic [7:0] sbox_outputs [0:3];
	
	sbox_sync s0(inword[0], clk, sbox_outputs[0]);
	sbox_sync s1(inword[1], clk, sbox_outputs[1]);
	sbox_sync s2(inword[2], clk, sbox_outputs[2]);
	sbox_sync s3(inword[3], clk, sbox_outputs[3]);
	
	always_ff @(posedge clk) begin
		outword = {sbox_outputs[0], sbox_outputs[1], sbox_outputs[2], sbox_outputs[3]};
	end
endmodule

module subword_tb (input logic [7:0] inword [0:3], output logic [7:0] outword [0:3]);
	logic int_osc;
	HSOSC #(.CLKHF_DIV(2'b01)) 
         hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc));
	
	subword dut(inword, int_osc, outword);
endmodule


// Main FSM for key scheduling and encryption for AES-128
module AES_FSM (
	input logic [127:0] key,
	input logic [127:0] plaintext,
	input logic clk,
	output logic done,
	output logic [127:0] result
);
	logic [127:0] ciphertext;
	logic [3:0] round;
	logic [31:0] w [0:3];
	logic [31:0] oldw [0:3];
	logic [31:0] rottemp;
	logic [31:0] subtemp;
	logic [7:0] rcon [0:10] = '{8'h01, 8'h02, 8'h04, 8'h08, 8'h10, 8'h20, 8'h40, 8'h80, 8'h1B, 8'h36, 8'h00};
	logic [127:0] tempSB;
	logic [127:0] tempSR;
	logic [127:0] tempMC;
	logic [127:0] tempARK;
	logic [11:0] counter;
	
	logic [4:0] state, nextstate;
	parameter S0 = 5'b0;
	parameter S1 = 5'b1;
	parameter S2 = 5'b10;
	parameter S3 = 5'b11;
	parameter S4 = 5'b100;
	parameter S5 = 5'b101;
	parameter S6 = 5'b110;
	parameter S7 = 5'b111;
	parameter S8 = 5'b1000;
	parameter S9 = 5'b1001;
	parameter S10 = 5'b1010;
	parameter S11 = 5'b1011;
	parameter S12 = 5'b1100;
	parameter S13 = 5'b1101;
	parameter S14 = 5'b1110;
	parameter S15 = 5'b1111;
	parameter S16 = 5'b10000;
	parameter S17 = 5'b10001;
	parameter S18 = 5'b10010;
	parameter S19 = 5'b10011;
	parameter S20 = 5'b10100;
	parameter S21 = 5'b10101;
	parameter S22 = 5'b10110;
	parameter S23 = 5'b10111;

	
	always_ff @(posedge clk) state <= nextstate;

	always_comb
		case (state)
			S0: nextstate = S22;
			S1: nextstate = S2;
			S2: nextstate = S3;
			S3: nextstate = S4;
			S4: nextstate = S5;
			S5: nextstate = S6;
			S6: nextstate = S7;
			S7: nextstate = S8;
			S8: nextstate = S9;
			S9: nextstate = S10;
			S10: nextstate = S11;
			S11: nextstate = S12;
			S12: nextstate = S17;
			S13: nextstate = S19;
			S14: nextstate = S20;
			S15: nextstate = S21;
			S16: nextstate = S16;
			S17: nextstate = S18;
			S18: nextstate = S13;
			S19: if (round == 10) nextstate = S15;
				 else nextstate = S14;
			S20: nextstate = S15;
			S21: if (round == 10) nextstate = S16;
			     else nextstate = S2;
			S22: if (counter == 2000) nextstate = S23;
				 else nextstate = S22;
			S23: nextstate = S1;
			default: nextstate = S0;
		endcase
		
	rotword rot ('{w[3][31:24], w[3][23:16], w[3][15:8], w[3][7:0]}, '{rottemp[31:24], rottemp[23:16], rottemp[15:8], rottemp[7:0]});
	subword sub ('{w[3][31:24], w[3][23:16], w[3][15:8], w[3][7:0]}, clk, '{subtemp[31:24], subtemp[23:16], subtemp[15:8], subtemp[7:0]});
	addroundkey ark(ciphertext, {w[0], w[1], w[2], w[3]}, tempARK);
	subbytes sb(ciphertext, clk, tempSB);
	shiftrows sr(ciphertext, tempSR);
	mixcolumns mc(ciphertext, tempMC);
	
	always_ff @(posedge clk)
		case (state)
			S0: counter <= 0;
			S22: counter <= counter + 1;
			S23: begin
				done <= 0;
				round <= 0;					
				w[0] <= key[127:96];
				w[1] <= key[95:64];
				w[2] <= key[63:32];
				w[3] <= key[31:0];
				ciphertext <= plaintext;
			end
			S1: ciphertext <= tempARK;
			S2: begin
				round <= round + 1;
				oldw[0] <= w[0];
				oldw[1] <= w[1];
				oldw[2] <= w[2];
				oldw[3] <= w[3];
			end
			S3: w[3] <= rottemp;
			S6: w[3] <= subtemp;
			S7: w[0] <= oldw[0] ^ w[3] ^ (rcon[round - 1] << 24); 
			S8: w[1] <= oldw[1] ^ w[0];
			S9: w[2] <= oldw[2] ^ w[1];
			S10: w[3] <= oldw[3] ^ w[2];
			S11: ciphertext <= tempSB;
			S13: ciphertext <= tempSR;
			S14: ciphertext <= tempMC;
			S15: ciphertext <= tempARK;
			S16: begin
				done <= 1;
				result <= ciphertext;
			end
		endcase
endmodule

module AES_FSM_tb (
	input logic [127:0] key,
	input logic [127:0] plaintext,
	output logic done,
	output logic [127:0] result
);
	logic int_osc;
	HSOSC #(.CLKHF_DIV(2'b01)) 
         hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc));
	AES_FSM dut(key, plaintext, int_osc, done, result);
endmodule
				
	
	
	
	
				
	
	