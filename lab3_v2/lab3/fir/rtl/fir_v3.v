module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    // AXILite
    // write
    output                        awready,
    output                        wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    
    // read
    output                       arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output                        rvalid,
    output    [(pDATA_WIDTH-1):0] rdata,

    // AXI-Stream
    // Input data 
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                      ss_tready, 
    // Output data
    input   wire                     sm_tready, 
    output  reg                      sm_tvalid, 
    output  reg  [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                      sm_tlast, 
    
    // bram for tap RAM (bram11) this use axilite
    output  reg  [3:0]               tap_WE,
    output  reg                      tap_EN,
    output  reg  [(pDATA_WIDTH-1):0] tap_Di,
    output  reg  [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM (bram12) this use stream
    output  reg  [3:0]               data_WE,
    output  reg                      data_EN,
    output  reg  [(pDATA_WIDTH-1):0] data_Di,
    output  reg  [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

    // write your code here!
    // parameter
    parameter S_FIR_IDLE = 0;
    parameter S_DATA_RST = 1;
    parameter S_FIR_WAIT = 2;
    parameter S_FIR_CALC = 3;
    parameter S_FIR_OUT  = 4;
    localparam AXILITE_IDLE   = 3'b000;
    localparam AXILITE_R_ADDR = 3'b100;
    localparam AXILITE_R_WAIT = 3'b101;
    localparam AXILITE_R_DATA = 3'b110;
    localparam AXILITE_W_ADDR = 3'b001;
    localparam AXILITE_W_DATA = 3'b010;

    reg [3:0] state_FIR, state_FIR_nxt;

    reg [(pADDR_WIDTH-1) : 0] tap_A_nxt;
    reg [(pDATA_WIDTH-1) : 0] tap_Di_nxt;
    reg [              3 : 0] tap_WE_nxt;
    reg                       tap_EN_nxt;
    reg [(pADDR_WIDTH-1) : 0] dataWriteAddr, dataWriteAdddr_nxt;
    reg [(pADDR_WIDTH-1) : 0] dataReadAddr, dataReadAddr_nxt;
    reg [              4 : 0] dataRam_rst_cnt;
    reg last_sig, last_sig_nxt; // remember last data signal
    reg [(pDATA_WIDTH-1) : 0] Xn, Yn, Hn, Yn_nxt;

    // Configuration register
    // according 
    reg ap_start, ap_start_nxt;
    reg ap_done, ap_done_nxt;
    reg ap_idle, ap_idle_nxt;
    reg [3:0] FIR_ctr, FIR_ctr_nxt;

    reg [2:0] state_axilite, state_axilite_nxt;

    reg [31:0] in_buf, in_buf_nxt, dataRAM_buf, dataRAM_buf_nxt;
    reg in_buf_full, in_buf_full_nxt;


// axilite signal
// write
    // output  reg                      awready,
    // output  reg                      wready,
    // input   wire                     awvalid,
    // input   wire [(pADDR_WIDTH-1):0] awaddr,
    // input   wire                     wvalid,
    // input   wire [(pDATA_WIDTH-1):0] wdata,
    
// read
    // output  wire                     arready,
    // input   wire                     rready,
    // input   wire                     arvalid,
    // input   wire [(pADDR_WIDTH-1):0] araddr,
    // output  reg                      rvalid,
    // output  reg  [(pDATA_WIDTH-1):0] rdata,

// 
// Jiin: basically axilite read/write won't happen simulataneously because it is the CPU generates the axilite read/write
// Jiin: If you want to handle read/write simultaneously, take care tap RAM read/write address
// ----
// we can do read and write on axilite simultaneously.
// we can get write address only when axilite state in idle (waiting for address)
assign awready = (state_axilite == AXILITE_W_ADDR) && ap_idle && !ap_done;

// we can get write data only when axilite state in data (waiting for data)
assign wready =  (state_axilite == AXILITE_W_DATA) && (state_FIR == S_FIR_IDLE);

// we can read address when axilite read state is in idle.
assign arready = (state_axilite == AXILITE_R_ADDR);

// rvalid raise when read data is prepared, which means state_axilite_read is in AXILITE_READ_DATA state
// if software want to read config, then set rvalid to 1 since configs are not in tap RAM.

// Jiin: In your testbench, do you deasserts araddr when sampled arready?
// Jiin: araddr is not valid after arready
// Jiin: you should use latched araddr
assign rvalid = (araddr == 12'h0) ? 1 : (state_axilite == AXILITE_R_DATA);

// rdata 
// Jiin: araddr is not valid in  state AXILITE_READ_DATA 
assign rdata = (araddr == 12'h0) ? {29'b0, ap_idle, ap_done, ap_start} :  // need to consider read config signal or tap RAM
            //    (araddr == 12'h10) ? data_length : // actually no need data_length
               (!ap_idle) ? 'hffffffff : tap_Do;   // need return ffffffff if engine is active

// First, write coefficient to tap RAM
    // tap_A:  when axilite handshake, update the value; else, remain the value
    // tap_Di: same as tap_A
    // tap_WE: write enable is when writing coefficient
    // tap_EN: enable when writing or when doing FIR (read coefficient)


always@(*) begin
    case (state_axilite)
        AXILITE_IDLE:   state_axilite_nxt = (awvalid) ? AXILITE_W_ADDR : (arvalid ? AXILITE_R_ADDR : AXILITE_IDLE);
        AXILITE_W_ADDR: state_axilite_nxt = AXILITE_W_DATA;
        AXILITE_W_DATA: state_axilite_nxt = (wvalid) ? AXILITE_IDLE : AXILITE_W_DATA;
        AXILITE_R_ADDR: state_axilite_nxt = AXILITE_R_WAIT;
        AXILITE_R_WAIT: state_axilite_nxt = AXILITE_R_DATA;
        AXILITE_R_DATA: state_axilite_nxt = (rvalid) ? AXILITE_IDLE : AXILITE_R_DATA;
        default:        state_axilite_nxt = AXILITE_IDLE;
    endcase
end

// Jiin: draw the axilite access timing and investigate how you can reduce the axilite read/write access timing
// Jiin: can you do awready/wready at the same cycle at awvalid/wvalid
//       arready at the same cycle as arvalid
//       rvalid one cycle after arvalid
// 
always@(*) begin
    // tap_A_nxt  = (awready && awvalid) ? awaddr : tap_A;
    // for tap_A, we need assign for read and write respectively.
    //     if write, then write address is valid when handshaking
    //     if read from tb, then read address is valid when handshaking and ap shoulde be idle
    //     if read from FIR,  or when doing FIR
    tap_A_nxt = (awready && awvalid) ? {5'b0, awaddr[6], awaddr[4:0]} : // write request from tb
                (arready && arvalid) ? {5'b0, araddr[6], araddr[4:0]} : // read request from tb
                (state_FIR == S_FIR_CALC && FIR_ctr == 10) ? {0, 2'b00} :
                (state_FIR_nxt == S_FIR_CALC) ? {FIR_ctr_nxt+1, 2'b00}:
                tap_A; // otherwise, read request from FIR engine

    tap_Di_nxt = (wready && wvalid) ? wdata : tap_Di;
    tap_WE_nxt = (wready && wvalid && awaddr != 12'h00 && awaddr != 12'h10) ? 4'b1111 : 4'b0;
    // tap enable when write handshake or read handshake or FIR executing
    tap_EN_nxt = (wready && wvalid) || (state_axilite == AXILITE_R_WAIT) || (state_FIR_nxt == S_FIR_CALC);
end

always@(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        tap_A <= 0;
        tap_Di <= 0;
        tap_WE <= 0;
        tap_EN <= 0;
        state_axilite <= AXILITE_IDLE;
    end else begin
        tap_A <= tap_A_nxt;
        tap_Di <= tap_Di_nxt;
        tap_WE <= tap_WE_nxt;
        tap_EN <= tap_EN_nxt;
        state_axilite <= state_axilite_nxt;
    end
end


// Sending Data from Stream to Data RAM
// We have     
    // output  reg  [3:0]               data_WE,
    // output  reg                      data_EN,
    // output  reg  [(pDATA_WIDTH-1):0] data_Di,
    // output  reg  [(pADDR_WIDTH-1):0] data_A,
    // input   wire [(pDATA_WIDTH-1):0] data_Do,
// for data RAM.
// data_Di: 

// Write pointer (dataWriteAddr)
// reference: https://github.com/holyuming/Verilog-FIR/tree/main
always @(*) begin
    // Update Write pointer when Stream-In
    // we store data in dataRAM when state_FIR is S_FIR_OUT
    if (state_FIR == S_FIR_OUT) begin
        if (dataWriteAddr == 10)    dataWriteAdddr_nxt = 0;
        else                        dataWriteAdddr_nxt = dataWriteAddr + 1;
    end
    else begin
        dataWriteAdddr_nxt = dataWriteAddr;
    end
end

// dataReadAddr, need to rotate read pointer to do convolution
// reference: https://github.com/holyuming/Verilog-FIR/tree/main

always @(*) begin
    // we should get the second data in the window in second cycle (first data is stream in)
    if ((state_FIR == S_FIR_OUT || (state_FIR == dataRam_rst_cnt && dataRam_rst_cnt == 10 ) || state_FIR == S_FIR_WAIT) && in_buf_full) begin
        dataReadAddr_nxt = (dataWriteAddr == 9) ? 0 : (dataWriteAddr == 10) ? 1 : dataWriteAddr + 2;
    end
    else if (state_FIR == S_FIR_CALC) begin
        dataReadAddr_nxt = (dataReadAddr == 10) ? 0 : dataReadAddr + 1;
    end
    else begin
        dataReadAddr_nxt = dataReadAddr;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        dataRam_rst_cnt <= 0;
        dataWriteAddr   <= 0;
        dataReadAddr    <= 1;
    end
    else begin 
        dataRam_rst_cnt <= (state_FIR_nxt != state_FIR) ? 0 : dataRam_rst_cnt + 1;
        dataWriteAddr   <= dataWriteAdddr_nxt;
        dataReadAddr    <= dataReadAddr_nxt;
    end
end

// data RAM interface
// reference: https://github.com/holyuming/Verilog-FIR/tree/main
always @(*) begin
    // default
    data_WE = 0;
    data_A = 0;
    data_Di = 0;
    data_EN = 1;

    if (state_FIR == S_FIR_CALC) begin
        data_EN = 1;
        data_A = (dataReadAddr_nxt << 2);   // This shifter (<< 2) could be share other where else
    end
    else if (state_FIR == S_DATA_RST) begin // reset all data in dataRAM
        data_EN = 1;
        data_A = (dataRam_rst_cnt << 2);
        data_WE = 4'b1111;
        data_Di = 32'd0;
    end
    else if (state_FIR == S_FIR_OUT) begin // Write data into dataRAM
        data_EN = 1;
        data_A = (dataWriteAddr << 2);
        data_WE = 4'b1111;
        data_Di = dataRAM_buf;
    end
end

// Stream interface, we have
// Input data 
    // input   wire                     ss_tvalid, 
    // input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    // input   wire                     ss_tlast, 
    // output  wire                     ss_tready, 
// Output data
    // input   wire                     sm_tready, 
    // output  wire                     sm_tvalid, 
    // output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    // output  wire                     sm_tlast,


always@(*) begin
    // ss_tready: we can get data when FIR machine is ready for stream in
    ss_tready = (!in_buf_full);
    if      (state_FIR == S_FIR_IDLE)  last_sig_nxt = 0;
    else if (state_FIR == S_FIR_CALC)  last_sig_nxt = ss_tlast; //TODO
    else                               last_sig_nxt = last_sig;

    // only assign value to stream out signals when in output stage
    sm_tvalid = (state_FIR == S_FIR_OUT);
    sm_tdata  = (state_FIR == S_FIR_OUT) ? Yn : 0;
    sm_tlast  = (state_FIR == S_FIR_OUT) ? last_sig_nxt : 0;

end


always@(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        last_sig <= 0;
    end else begin
        last_sig <= last_sig_nxt;
    end
end

// configuration register access protocol
always@(*) begin
    // ap_start
    if      (awaddr == 'h00 && wready && wvalid && wdata == 1) ap_start_nxt = wdata;     // Host program ap_start = 1
    else if (!ap_idle)                                  ap_start_nxt = 0;         // FSM exit idle, reset ap_start to 0
    else                                               ap_start_nxt = ap_start;

    // ap_idle
    if      (awaddr == 'h00 && wready && wvalid && wdata == 1) ap_idle_nxt = 0; // set to 0 when ap_start is sampled (have same logic as ap_start)
    else if (state_FIR == S_FIR_OUT && last_sig)             ap_idle_nxt = 1; // set to 1 when FIR engine process the last data and last data is transferred
    else                                                     ap_idle_nxt = ap_idle;
    
    // ap_done
    if      (state_FIR == S_FIR_OUT && last_sig) ap_done_nxt = 1; // assert when engine completes the last data and last data is transferred
    else if (state_FIR == S_FIR_CALC)            ap_done_nxt = 0; // 
    else                                         ap_done_nxt = ap_done;
end



// Jiin: No need for tap programmed checker  

always@(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        ap_start <= 0;
        ap_done  <= 0;
        ap_idle  <= 1; // ap_idle is set to 1 when reset
    end else begin
        ap_start <= ap_start_nxt;
        ap_done  <= ap_done_nxt;
        ap_idle  <= ap_idle_nxt;
    end
end



// FIR fsm
// 10/17 teacher said FIR engine should not have interface signal
// reference: https://github.com/holyuming/Verilog-FIR/tree/main

// FIR 
always@(*) begin
    if      (state_FIR == S_FIR_CALC) FIR_ctr_nxt = FIR_ctr + 1;
    else                              FIR_ctr_nxt = 0;
end

// Jiin: should decouple the state_FIR from ss, sm bus interface
// Jiin: what is the Y throughput based on the state machine

// input buffer for ss_tdata
// reg [pDATA_WIDTH-1:0] ss_tdata_buf, ss_tdata_buf_nxt;
// reg ss_tdata_buf_is_empty, ss_tdata_buf_is_empty_nxt;


always@(*) begin
    if (!axis_rst_n) begin
        in_buf_nxt = 0;
        in_buf_full_nxt = 1;
    end else if (state_FIR == S_FIR_IDLE && !in_buf_full) begin
        in_buf_nxt = ss_tdata;
        in_buf_full_nxt = 1;
    end else if (state_FIR == S_FIR_IDLE && in_buf_full) begin
        in_buf_nxt = in_buf;
        in_buf_full_nxt = 1;
    end else if (ss_tvalid && !in_buf_full) begin // receive data when buffer is not full
        in_buf_nxt = ss_tdata;
        in_buf_full_nxt = 1;
    end else if (state_FIR == S_DATA_RST && dataRam_rst_cnt == 10 && in_buf_full) begin
        in_buf_nxt = 0;
        in_buf_full_nxt = 0;
    end else if (state_FIR == S_FIR_OUT && in_buf_full) begin
        in_buf_nxt = 0;
        in_buf_full_nxt = 0;
    end else begin
        in_buf_nxt = in_buf;
        in_buf_full_nxt = in_buf_full;
    end
end

always@(*) begin
    if (state_FIR == S_FIR_CALC && FIR_ctr == 1) dataRAM_buf_nxt = in_buf; // move new data to dataRAM buffer at the first cycle
    else dataRAM_buf_nxt = dataRAM_buf;
end


always@(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        FIR_ctr <= 0;
        in_buf <= 0;
        in_buf_full <= 1;
        dataRAM_buf <= 0;
    end else begin
        FIR_ctr <= FIR_ctr_nxt;
        in_buf  <= in_buf_nxt;
        in_buf_full <= in_buf_full_nxt;
        dataRAM_buf <= dataRAM_buf_nxt;
    end
end

always @(*) begin
    case (state_FIR)
        S_FIR_IDLE:   state_FIR_nxt = (ap_start) ?  S_DATA_RST : S_FIR_IDLE;
        S_DATA_RST:   state_FIR_nxt = (dataRam_rst_cnt == 10) ? (in_buf_full ? S_FIR_CALC : S_FIR_WAIT) : S_DATA_RST;
        S_FIR_WAIT:   state_FIR_nxt = (in_buf_full) ? S_FIR_CALC : S_FIR_WAIT;
        S_FIR_CALC:   state_FIR_nxt = (FIR_ctr == 10) ? S_FIR_OUT : S_FIR_CALC; // 11 states for calculating FIR (n == 11)
        S_FIR_OUT: begin
            if (last_sig)
                state_FIR_nxt = S_FIR_IDLE; // reset
            else if (sm_tready) 
                state_FIR_nxt = (in_buf_full) ? S_FIR_CALC : S_FIR_WAIT;
            else 
                state_FIR_nxt = S_FIR_OUT;  // wait for handshakes
        end
        default: state_FIR_nxt = S_FIR_IDLE;
    endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)    state_FIR   <= S_FIR_IDLE;
    else                state_FIR   <= state_FIR_nxt;
end


// Get the Xn, Hn from dataRAM & tapRAM
// Jiin:  how  to separate the mulitiplication (*) and addition (+) running in different cycle
// 
always @(*) begin
    Xn = FIR_ctr == 10 ? dataRAM_buf : data_Do;
    Hn = tap_Do;
    if      (state_FIR == S_FIR_OUT)      Yn_nxt = 0; // reset accumulator
    else if (state_FIR == S_DATA_RST)      Yn_nxt = 0; // reset accumulator
    else if (state_FIR_nxt == S_FIR_CALC) Yn_nxt = Yn + (Xn * Hn); // accumulate Xn * Hn
    else    Yn_nxt = Yn;
end

// Accumulator and Multiplier (1 adder & 1 multiplier for FIR)
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)
        Yn <= 0;
    else begin
        Yn <= Yn_nxt;
    end
end



// Sequential block


endmodule