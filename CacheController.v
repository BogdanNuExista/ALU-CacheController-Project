module CacheController (
    input clk,
    input reset,
    input [31:0] address,
    input [31:0] write_data,
    input read,
    input write,
    output reg [31:0] read_data,
    output reg hit,
    output reg mem_read,
    output reg mem_write,
    output reg [31:0] mem_address,
    output reg [31:0] mem_write_data,
    input [31:0] mem_read_data,
    input mem_ready
);
    parameter CACHE_SIZE = 32 * 1024; // 32 KB
    parameter BLOCK_SIZE = 64;        // 64 bytes
    parameter NUM_SETS = 128;         // Number of sets
    parameter ASSOCIATIVITY = 4;      // 4-way set associative
    parameter NUM_BLOCKS = CACHE_SIZE / BLOCK_SIZE;

    // Calculating indices
    parameter INDEX_BITS = $clog2(NUM_SETS);
    parameter OFFSET_BITS = $clog2(BLOCK_SIZE);
    parameter TAG_BITS = 32 - INDEX_BITS - OFFSET_BITS;

    // Cache block structure
    reg [TAG_BITS-1:0] tags [NUM_SETS-1:0][ASSOCIATIVITY-1:0];
    reg [BLOCK_SIZE*8-1:0] data [NUM_SETS-1:0][ASSOCIATIVITY-1:0];
    reg valid [NUM_SETS-1:0][ASSOCIATIVITY-1:0];
    reg dirty [NUM_SETS-1:0][ASSOCIATIVITY-1:0]; // Dirty bits for write-back policy
    reg [1:0] lru [NUM_SETS-1:0][ASSOCIATIVITY-1:0]; // LRU bits for replacement policy

    // State definitions
    localparam IDLE = 3'b000;
    localparam READ_HIT = 3'b001;
    localparam READ_MISS = 3'b010;
    localparam WRITE_HIT = 3'b011;
    localparam WRITE_MISS = 3'b100;
    localparam EVICT = 3'b101;
    localparam FETCH = 3'b110; // State to fetch data from memory

    reg [2:0] state, next_state;
    
    // Internal signals
    reg [INDEX_BITS-1:0] index;
    reg [TAG_BITS-1:0] tag;
    reg [OFFSET_BITS-1:0] offset;
    reg hit_detected;
    integer i, j;

    // LRU update function
    task update_lru(input [INDEX_BITS-1:0] set, input [1:0] way);
        begin
            // Update LRU counters
            for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
                if (lru[set][i] < lru[set][way])
                    lru[set][i] = lru[set][i] + 1;
            end
            lru[set][way] = 0;
        end
    endtask

    // Address decomposition
    always @(*) begin
        index = address[OFFSET_BITS + INDEX_BITS - 1: OFFSET_BITS];
        tag = address[31: 32 - TAG_BITS];
        offset = address[OFFSET_BITS-1:0];
    end

    // FSM State Transitions
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= IDLE;
        else
            state <= next_state;
    end

    // FSM Next State Logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (read || write) begin
                    hit_detected = 0;
                    for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
                        if (valid[index][i] && tags[index][i] == tag) begin
                            hit_detected = 1;
                        end
                    end
                    if (hit_detected) begin
                        if (read)
                            next_state = READ_HIT;
                        else
                            next_state = WRITE_HIT;
                    end else begin
                        if (read)
                            next_state = READ_MISS;
                        else
                            next_state = WRITE_MISS;
                    end
                end
            end
            READ_HIT: next_state = IDLE;
            WRITE_HIT: next_state = IDLE;
            READ_MISS: next_state = FETCH;
            WRITE_MISS: next_state = FETCH;
            FETCH: begin
                if (mem_ready)
                    next_state = EVICT;
            end
            EVICT: next_state = IDLE;
        endcase
    end

    // FSM Output Logic
    always @(posedge clk) begin
        if (reset) begin
            // Initialize cache
            for (i = 0; i < NUM_SETS; i = i + 1)
                for (j = 0; j < ASSOCIATIVITY; j = j + 1) begin
                    valid[i][j] <= 0;
                    dirty[i][j] <= 0;
                    lru[i][j] <= j;
                end
            read_data <= 0;
            hit <= 0;
            mem_read <= 0;
            mem_write <= 0;
            mem_address <= 0;
            mem_write_data <= 0;
        end else begin
            case (state)
                IDLE: begin
                    hit <= 0;
                end
                READ_HIT: begin
                    for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
                        if (valid[index][i] && tags[index][i] == tag) begin
                            read_data <= data[index][i][offset*8 +: 32];
                            update_lru(index, i);
                            hit <= 1;
                        end
                    end
                end
                WRITE_HIT: begin
                    for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
                        if (valid[index][i] && tags[index][i] == tag) begin
                            data[index][i][offset*8 +: 32] <= write_data;
                            dirty[index][i] <= 1; // Mark block as dirty
                            update_lru(index, i);
                            hit <= 1;
                        end
                    end
                end
                READ_MISS, WRITE_MISS: begin
                    mem_read <= 1;
                    mem_address <= address & ~(BLOCK_SIZE-1);
                end
                EVICT: begin
                    for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
                        if (lru[index][i] == (ASSOCIATIVITY - 1)) begin
                            if (dirty[index][i]) begin
                                mem_write <= 1;
                                mem_address <= {tags[index][i], index, {OFFSET_BITS{1'b0}}};
                                mem_write_data <= data[index][i];
                            end
                            tags[index][i] <= tag;
                            valid[index][i] <= 1;
                            dirty[index][i] <= 0;
                            update_lru(index, i);
                        end
                    end
                end
                FETCH: begin
                    if (mem_ready) begin
                        for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
                            if (lru[index][i] == (ASSOCIATIVITY - 1)) begin
                                data[index][i] <= mem_read_data;
                                valid[index][i] <= 1;
                                dirty[index][i] <= 0;
                                tags[index][i] <= tag;
                                update_lru(index, i);
                            end
                        end
                        mem_read <= 0;
                        mem_write <= 0;
                    end
                end
            endcase
        end
    end
endmodule




////////////////////////////////////////////////////////////////////////////////////////////////////////////////


module main_tb;
    reg clk;
    reg reset;
    reg [31:0] address;
    reg [31:0] write_data;
    reg read;
    reg write;
    wire [31:0] read_data;
    wire hit;
    wire mem_read;
    wire mem_write;
    wire [31:0] mem_address;
    wire [31:0] mem_write_data;
    reg [31:0] mem_read_data;
    reg mem_ready;

    // Instantiate the cache controller
    CacheController uut (
        .clk(clk),
        .reset(reset),
        .address(address),
        .write_data(write_data),
        .read(read),
        .write(write),
        .read_data(read_data),
        .hit(hit),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_address(mem_address),
        .mem_write_data(mem_write_data),
        .mem_read_data(mem_read_data),
        .mem_ready(mem_ready)
    );

    // Clock generation
    always #5 clk = ~clk;

    // Test procedure
    initial begin
        // Initialize signals
        clk = 0;
        reset = 1;
        address = 0;
        write_data = 0;
        read = 0;
        write = 0;
        mem_ready = 0;
        mem_read_data = 0;

        // Apply reset
        #10;
        reset = 0;
        $display("Simulation started.");

        // Test Case 1: Read miss, fetch from memory
        $display("");
        $display("Test Case 1: Read miss, fetch from memory");
        address = 32'h0000_0040;
        read = 1;
        #10;
        read = 0;
        #10;
        mem_ready = 1;
        mem_read_data = 32'hDEAD_BEEF;
        #10;
        mem_ready = 0;
        #10;
        $display("Data read from cache: %h, Hit: %b", read_data, hit);

        // Test Case 2: Read hit
        $display("");
        $display("Test Case 2: Read hit");
        address = 32'h0000_0040;
        read = 1;
        #10;
        read = 0;
        #10;
        $display("Data read from cache: %h, Hit: %b", read_data, hit);

        // Test Case 3: Write miss, fetch from memory, then write
        $display("");
        $display("Test Case 3: Write miss, fetch from memory, then write");
        address = 32'h0000_0080;
        write_data = 32'hCAFEBABE;
        write = 1;
        #10;
        write = 0;
        #10;
        mem_ready = 1;
        mem_read_data = 32'hF00D_F00D;
        #10;
        mem_ready = 0;
        #10;
        $display("Write data to cache: %h, Hit: %b", write_data, hit);

        // Test Case 4: Write hit
        $display("");
        $display("Test Case 4: Write hit");
        address = 32'h0000_0080;
        write_data = 32'hDEADBEEF;
        write = 1;
        #10;
        write = 0;
        #10;
        $display("Write data to cache: %h, Hit: %b", write_data, hit);

        // Test Case 5: Read hit after write
        $display("");
        $display("Test Case 5: Read hit after write");
        address = 32'h0000_0080;
        read = 1;
        #10;
        read = 0;
        #10;
        $display("Data read from cache: %h, Hit: %b", read_data, hit);

        // Finish simulation
        $display("");
        $display("Simulation finished.");
    end
endmodule

