////////////////////// Main Module /////////////////////

module main(y1,y2,y3,cout,carry_out,m,sel,a,b,x,y,A,B,j,k,cin,Op);

output reg [15:0]y1;
output reg [31:0]y2;
output reg [15:0]y3;
output cout;
output carry_out;
output m;
input [2:0]sel;
input [15:0]a;
input [15:0]b;
input [15:0]x;
input [15:0]y;
input [15:0]A;
input [15:0]B;
input [15:0]j;
input [15:0]k;
input cin;
input Op;

wire [15:0]s;

wire [15:0]sum;

wire [15:0]buff1;
wire [15:0]buff2;

wire [15:0]rightshift;
wire [15:0]leftshift;

wire [15:0] quotent;
wire [15:0] reminder;

////////////////////////////////////////////////////

CLAdder c1(s,cout,a,b,cin);
subtractor sub1(sum,carry_out,m,x,y,Op);
multiplier m1(buff1,buff2,A,B);
shifter shift1(leftshift,rightshift,j,k);
division_module divmod(A,B,quotent,reminder);

always@*
    begin
        if(sel==3'b000)
        begin
             y1[15:0] = s[15:0];
             y2[31:0] = 31'bz;
             y3[15:0] = 16'bz; 
        end
        else if(sel==3'b001)
        begin
             y1[15:0] = sum[15:0];
             y2 [15:0] = 16'bz;
             y3[15:0] = 16'bz;  
        end
        else if(sel==3'b010)
        begin
             y2[31:0] = {buff1[15:0],buff2[15:0]};   
             y1[15:0] = 16'bz;
             y3[15:0] = 16'bz;
        end
        else if(sel==3'b011)
        begin   
             y3[15:0] = leftshift[15:0];
             y1[15:0] = rightshift[15:0];
             y2 [15:0] = 16'bz;
        end
        else if(sel==3'b100)
        begin
             y1[15:0]=quotent[15:0];
             y3[15:0]=reminder[15:0];
             y2[31:0]=31'bz;
        end
    end
endmodule
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////// ADDER //////////////////////
module CLAdder(s, cout, a,b, cin);
output [15:0]s;
output cout;
input cin;

input [15:0]a;
input [15:0]b; 

wire cout1;                                             
cladder4 cla1(s[3:0],cout1,a[3:0],b[3:0],cin);         
cladder4 cla2(s[7:4],cout2,a[7:4],b[7:4],cout1);           
cladder4 cla3(s[11:8],cout3,a[11:8],b[11:8],cout2);       
cladder4 cla4(s[15:12],cout,a[15:12],b[15:12],cout3);        
endmodule
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module cladder4(output [3:0]s, c3, input [3:0]a, [3:0]b, cin);

PFAdder pfa1(s[0],p0,g0,a[0],b[0],cin);              
cla_logic clogic1(c0,p0,g0,cin);                  

PFAdder pfa2(s[1],p1,g1,a[1],b[1],c0);
cla_logic clogic2(c1,p1,g1,c0);

PFAdder pfa3(s[2],p2,g2,a[2],b[2],c1);
cla_logic clogic3(c2,p2,g2,c1);

PFAdder pfa4(s[3],p3,g3,a[3],b[3],c2);
cla_logic clogic4(c3,p3,g3,c2);
endmodule
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module PFAdder(output s,p,g, input a,b,cin);
xor(s,a,b,cin);                     
and(g,a,b);                         
xor(p,a,b);                           
endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////CLA Logic:

module cla_logic(output c,input p,g,cin);
wire w1;                    
and(w1,p,cin);             
or(c,g,w1);                 
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////// SUBTRACTOR ///////////////////////
module subtractor(sum, carry_out, m, x, y, Op);
   output [15:0] sum;  
   output      carry_out; 
   output      m;  
   input [15:0]        x;  
   input [15:0]        y;  
   input       Op; 
  
   wire C0,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15;
   wire B0,B1,B2,B3,B4,B5,B6,B7,B8,B9,B10,B11,B12,B13,B14,B15;
   
    xor(B0, y[0], Op);
    xor(B1, y[1], Op);
    xor(B2, y[2], Op);
    xor(B3, y[3], Op);
    xor(B4, y[4], Op);
    xor(B5, y[5], Op);
    xor(B6, y[6], Op);
    xor(B7, y[7], Op);
    xor(B8, y[8], Op);
    xor(B9, y[9], Op);
    xor(B10, y[10], Op);
    xor(B11, y[11], Op);
    xor(B12, y[12], Op);
    xor(B13, y[13], Op);
    xor(B14, y[14], Op);
    xor(B15, y[15], Op);
    xor(carry_out, C15, Op);    
    xor(m, C15, C14);    
  
    fulladder fa0(sum[0], C0, x[0], B0, Op);   
    fulladder fa1(sum[1], C1, x[1], B1, C0);
    fulladder fa2(sum[2], C2, x[2], B2, C1);
    fulladder fa3(sum[3], C3, x[3], B3, C2);   
    fulladder fa4(sum[4], C4, x[4], B4, C3);   
    fulladder fa5(sum[5], C5, x[5], B5, C4);   
    fulladder fa6(sum[6], C6, x[6], B6, C5);   
    fulladder fa7(sum[7], C7, x[7], B7, C6);   
    fulladder fa8(sum[8], C8, x[8], B8, C7);   
    fulladder fa9(sum[9], C9, x[9], B9, C8);   
    fulladder fa10(sum[10], C10, x[10], B10, C9);   
    fulladder fa11(sum[11], C11, x[11], B11, C10);   
    fulladder fa12(sum[12], C12, x[12], B12, C11);   
    fulladder fa13(sum[13], C13, x[13], B13, C12);   
    fulladder fa14(sum[14], C14, x[14], B14, C13);   
    fulladder fa15(sum[15], C15, x[15], B15, C14);   
endmodule
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module fulladder(S, Cout, A, B, Cin);
   output S;
   output Cout;
   input  A;
   input  B;
   input  Cin;
     wire   w1,w2,w3,w4;
     xor(w1, A, B);
   xor(S, Cin, w1);
   and(w2, A, B);  
   and(w3, A, Cin);
   and(w4, B, Cin);  
   or(Cout, w2, w3, w4);
endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////// MULTIPLIER ////////////////////////


module multiplier(buff1,buff2,A,B);

output reg [15:0] buff1;
output reg [15:0] buff2;
input [15:0] A;
input [15:0] B;

reg [15:0] C;
integer i;

always @* begin
    buff1 = 0;
    C = B;
    for(i=0; i<16; i=i+1) begin
        if(C[0] == 1) begin
            buff1 = buff1 + A;
            C = C >> 1;
            C[15] = buff1[0];
            buff1 = buff1 >> 1;
        end else if(C[0] == 0) begin
            C = C >> 1;
            C[15] = buff1[0];
            buff1 = buff1 >> 1;
        end
        buff2 = C;
    end       
end
endmodule


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////// SHIFTER /////////////////
module shifter(
    output reg [15:0] leftshift,
    output reg [15:0] rightshift,
    input [15:0] j,
    input [15:0] k);
always@(k)
begin
leftshift = j<<k;
rightshift <= j>>k;
end

endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////// DIVISION //////////////////////

module division_module(
    input [15:0] dividend, 
    input [15:0] divisor,  
    output reg [15:0] quotient,  
    output reg [15:0] remainder  
);

reg [15:0] dividend_reg;
reg [15:0] quotient_reg;
reg [15:0] remainder_reg;
reg [15:0] partial_remainder;

integer i;

always @* begin
    dividend_reg = dividend;
    quotient_reg = 0;
    remainder_reg = 0;

    for (i = 15; i >= 0; i = i - 1) begin
        partial_remainder = {remainder_reg, dividend_reg[i]}; 
        if (partial_remainder >= divisor) begin
            partial_remainder = partial_remainder - divisor; 
            quotient_reg[i] = 1'b1;
        end
        remainder_reg = partial_remainder;
    end
end

always @* begin
    quotient = quotient_reg;
    remainder = remainder_reg;
end

endmodule

/// TESTBENCH 

module main_tb;
  
reg [2:0]sel;
reg [15:0]a;
reg [15:0]b;
reg [15:0]x;
reg [15:0]y;
reg [15:0]A;
reg [15:0]B;
reg [15:0]j;
reg [15:0]k;
reg cin;
reg Op;

wire [15:0]y1;
wire [31:0]y2;
wire [15:0]y3;
wire cout;
wire carry_out;
wire m;

main uut(y1,y2,y3,cout,carry_out,m,sel,a,b,x,y,A,B,j,k,cin,Op);
initial begin
sel = 3'b000; // selectam operatia de adunare
a = 10;
b = 20;

cin = 0;
Op = 1;
#10;
$display("Adunand nr: %d + %d obtinem : %d",a,b,y1);

sel = 3'b001; // selectam operatia de scadere 

x = 25;
y = 11;

cin = 0;
Op = 1;
#10;
$display("Scazand nr: %d - %d obtinem : %d",x,y,y1);

sel = 3'b010; // selectam operatia de inmultire

A = 16'b0000000000001000;
B = 16'b0000000000001000;

cin = 0;
Op = 1;
#10;
$display("Inmultind nr:%d * %d obtinem : %d",A,B,y2);

sel = 3'b011; // selectam operatia de shiftare

j = 16'hff12;
k = 3;

cin = 0;
Op = 1;
#10;
$display("Shiftand numarul j=%d cu k=%d pozitii obinem: %d",j,k,y3);

sel = 3'b100;

A = 15;
B = 2;

cin = 0;
Op = 1;
#10;
$display("Impartind nr:%d / %d obtinem : %d rest %d",A,B,y1,y3);

end
endmodule


