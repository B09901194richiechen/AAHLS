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
parameter AXILITE_WRITE_IDLE = 2'd0;
parameter AXILITE_WRITE_ADDR = 2'd1;
parameter AXILITE_WRITE_DATA = 2'd2;

parameter AXILITE_READ_IDLE  = 2'd0;
parameter AXILITE_READ_ADDR  = 2'd1;
parameter AXILITE_READ_WAIT  = 2'd2;
parameter AXILITE_READ_DATA  = 2'd3;

// parameter S_FIR_IDLE = 4'd0;
// parameter S_FIR_SIN  = 4'd1;
// parameter S_FIR_CALC = 4'd2;

parameter S_FIR_IDLE = 12;
parameter S_DATA_RST = 15;
parameter S_FIR_WAIT = 14;
parameter S_FIR_SIN = 11;
parameter S_FIR_STOR = 13;
parameter S_FIR_S0 = 0;
parameter S_FIR_S1 = 1;
parameter S_FIR_S2 = 2;
parameter S_FIR_S3 = 3;
parameter S_FIR_S4 = 4;
parameter S_FIR_S5 = 5;
parameter S_FIR_S6 = 6;
parameter S_FIR_S7 = 7;
parameter S_FIR_S8 = 8;
parameter S_FIR_S9 = 9;
parameter S_FIR_SA = 10;
parameter S_FIR_OUT = 16;

reg [1:0] state_axilite_write, state_axilite_write_nxt;
reg [1:0] state_axilite_read, state_axilite_read_nxt;
reg [4:0] state_FIR, state_FIR_nxt;

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
// wire ap_start_nxt = 0;
reg ap_done, ap_done_nxt;
reg ap_idle, ap_idle_nxt;

reg data_length_tap_params_programmed_checker[0:11];
reg data_length_tap_params_programmed_checker_nxt[0:11];

integer i;

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

// we can do read and write on axilite simultaneously.
// we can get write address only when axilite state in idle (waiting for address)
assign awready = (state_axilite_write == AXILITE_WRITE_IDLE) && ap_idle && !ap_done;

// we can get write data only when axilite state in data (waiting for data)
assign wready =  (state_axilite_write == AXILITE_WRITE_DATA) && (state_FIR == S_FIR_IDLE);

// we can read address when axilite read state is in idle.
assign arready = (state_axilite_read == AXILITE_READ_ADDR) && (state_axilite_write == AXILITE_WRITE_IDLE);

// rvalid raise when read data is prepared, which means state_axilite_read is in AXILITE_READ_DATA state
// if software want to read config, then set rvalid to 1 since configs are not in tap RAM.
assign rvalid = (araddr == 12'h0) ? 1 : (state_axilite_read == AXILITE_READ_DATA);

// rdata 
assign rdata = (araddr == 12'h0) ? {29'b0, ap_idle, ap_done, ap_start} :  // need to consider read config signal or tap RAM
            //    (araddr == 12'h10) ? data_length : // actually no need data_length
               (!ap_idle) ? 'hffffffff : tap_Do;   // need return ffffffff if engine is active

// First, write coefficient to tap RAM
    // tap_A:  when axilite handshake, update the value; else, remain the value
    // tap_Di: same as tap_A
    // tap_WE: write enable is when writing coefficient
    // tap_EN: enable when writing or when doing FIR (read coefficient)

always @(*) begin
    case (state_axilite_write)
        AXILITE_WRITE_IDLE: state_axilite_write_nxt = (awvalid) ? AXILITE_WRITE_ADDR : AXILITE_WRITE_IDLE;
        AXILITE_WRITE_ADDR: state_axilite_write_nxt = AXILITE_WRITE_DATA;
        AXILITE_WRITE_DATA: state_axilite_write_nxt = (wvalid) ? AXILITE_WRITE_IDLE : AXILITE_WRITE_DATA;
        default:            state_axilite_write_nxt = AXILITE_WRITE_IDLE;
    endcase
end

always @(*) begin
    case (state_axilite_read)
        AXILITE_READ_IDLE: state_axilite_read_nxt = (arvalid && state_axilite_write == AXILITE_WRITE_IDLE) ? AXILITE_READ_ADDR : AXILITE_READ_IDLE;
        AXILITE_READ_ADDR: state_axilite_read_nxt = AXILITE_READ_WAIT;
        AXILITE_READ_WAIT: state_axilite_read_nxt = AXILITE_READ_DATA;
        AXILITE_READ_DATA: state_axilite_read_nxt = (rvalid) ? AXILITE_READ_IDLE : AXILITE_READ_DATA;
        default: state_axilite_read_nxt = AXILITE_READ_IDLE;
    endcase
end

always@(*) begin
    // tap_A_nxt  = (awready && awvalid) ? awaddr : tap_A;
    // for tap_A, we need assign for read and write respectively.
    //     if write, then write address is valid when handshaking
    //     if read from tb, then read address is valid when handshaking and ap shoulde be idle
    //     if read from FIR,  or when doing FIR
    tap_A_nxt = (awready && awvalid) ? {5'b0, awaddr[6], awaddr[4:0]} : // write request from tb
                (arready && arvalid) ? {5'b0, araddr[6], araddr[4:0]} : // read request from tb
                (S_FIR_S0 <= state_FIR_nxt && state_FIR_nxt < S_FIR_SA) ? {state_FIR_nxt+1, 2'b00} : // we have to left shift two bits before accessing to bram using real address.....
                tap_A; // otherwise, read request from FIR engine

    tap_Di_nxt = (wready && wvalid) ? wdata : tap_Di;
    tap_WE_nxt = (wready && wvalid && awaddr != 12'h00 && awaddr != 12'h10) ? 4'b1111 : 4'b0;
    // tap enable when write handshake or read handshake or FIR executing
    tap_EN_nxt = (wready && wvalid) || (state_axilite_read == AXILITE_READ_WAIT) || (S_FIR_S0 <= state_FIR_nxt && state_FIR_nxt <= S_FIR_SA);
    // tap_EN_nxt = (wready && wvalid) || (state_axilite_read == AXILITE_READ_WAIT) || (!ap_idle);

    // tap_EN_nxt = (state_axilite_write == AXILITE_WRITE_DATA) || (state_axilite_read == AXILITE_READ_IDLE) || ap_start;

end

always@(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        tap_A <= 0;
        tap_Di <= 0;
        tap_WE <= 0;
        tap_EN <= 0;
        state_axilite_read <= 0;
        state_axilite_write <= 0;
    end else begin
        tap_A <= tap_A_nxt;
        tap_Di <= tap_Di_nxt;
        tap_WE <= tap_WE_nxt;
        tap_EN <= tap_EN_nxt;
        state_axilite_read <= state_axilite_read_nxt;
        state_axilite_write <= state_axilite_write_nxt;
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
    if (state_FIR == S_FIR_SIN) begin
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
    if (state_FIR == S_FIR_SIN) begin
        dataReadAddr_nxt = (dataWriteAddr == 10) ? 0 : dataWriteAddr + 1;
    end
    else if (S_FIR_S0 <= state_FIR && state_FIR <= S_FIR_SA) begin
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

    if (S_FIR_S0 <= state_FIR_nxt && state_FIR_nxt <= S_FIR_SA) begin
        data_EN = 1;
        data_A = (dataReadAddr_nxt << 2);   // This shifter (<< 2) could be share other where else
    end
    else if (state_FIR == S_DATA_RST) begin // reset all data in dataRAM
        data_EN = 1;
        data_A = (dataRam_rst_cnt << 2);
        data_WE = 4'b1111;
        data_Di = 32'd0;
    end
    else if (state_FIR == S_FIR_SIN) begin // Write data into dataRAM
        data_EN = 1;
        data_A = (dataWriteAddr << 2);
        data_WE = 4'b1111;
        data_Di = ss_tdata;
    end
end

// reg [(pADDR_WIDTH-1) : 0] data_A_nxt;
// reg [(pDATA_WIDTH-1) : 0] data_Di_nxt;
// reg [              3 : 0] data_WE_nxt;
// reg                       data_EN_nxt;

// always@(*) begin
//     // data_A

//     // data_Di

//     // data_EN
//     if (state_FIR == DATA_RST || state_FIR == FIR_SIN || state_FIR == FIR_CALC) data_WE_nxt = 1;
//     else data_WE = 0;

//     // data_WE

// end
// always@(posedge axis_clk or negedge axis_rst_n) begin
//     if (!axis_rst_n) begin
//         data_A  <= 0;
//         data_Di <= 0;
//         data_WE <= 0;
//         data_EN <= 0;
//     end else begin
//         data_A  <= data_A _nxt;
//         data_Di <= data_Di_nxt;
//         data_WE <= data_WE_nxt;
//         data_EN <= data_EN_nxt;
//     end
// end


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
    ss_tready = (state_FIR == S_FIR_SIN);
    if      (state_FIR == S_FIR_IDLE) last_sig_nxt = 0;
    else if (state_FIR == S_FIR_SIN)  last_sig_nxt = ss_tlast;
    else                              last_sig_nxt = last_sig;

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
wire checked;
assign checked = data_length_tap_params_programmed_checker[0] & data_length_tap_params_programmed_checker[1] & data_length_tap_params_programmed_checker[2] & data_length_tap_params_programmed_checker[3] & data_length_tap_params_programmed_checker[4] & data_length_tap_params_programmed_checker[5] & data_length_tap_params_programmed_checker[6] & data_length_tap_params_programmed_checker[7] & data_length_tap_params_programmed_checker[8] & data_length_tap_params_programmed_checker[9] & data_length_tap_params_programmed_checker[10] & data_length_tap_params_programmed_checker[11];
always@(*) begin
    // ap_start
    if      (awaddr == 'h00 && wready && wvalid && wdata == 1 && (checked)) ap_start_nxt = wdata;     // Host program ap_start = 1
    else if (!ap_idle)                                  ap_start_nxt = 0;         // FSM exit idle, reset ap_start to 0
    else                                               ap_start_nxt = ap_start;

    // ap_idle
    if      (awaddr == 'h00 && wready && wvalid && wdata == 1 && (checked)) ap_idle_nxt = 0; // set to 0 when ap_start is sampled (have same logic as ap_start)
    else if (state_FIR == S_FIR_OUT && last_sig)             ap_idle_nxt = 1; // set to 1 when FIR engine process the last data and last data is transferred
    else                                                     ap_idle_nxt = ap_idle;
    
    // ap_done
    if      (state_FIR == S_FIR_OUT && last_sig) ap_done_nxt = 1; // assert when engine completes the last data and last data is transferred
    else if (state_FIR == S_FIR_SIN)             ap_done_nxt = 0; // 
    else                                       ap_done_nxt = ap_done;
end



// 
always@(*) begin
    data_length_tap_params_programmed_checker_nxt[11] = (last_sig) ? 0 : (awaddr == 'h10 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[11];
    // for (i=0;i<11;i=i+1) begin
    //     data_length_tap_params_programmed_checker_nxt[i] = (last_sig) ? 1'b0 : (awaddr == ('h20 + i << 2) && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[i];
    // end
    data_length_tap_params_programmed_checker_nxt[0] = (last_sig) ? 0 : (awaddr == 'h20 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[0];
    data_length_tap_params_programmed_checker_nxt[1] = (last_sig) ? 0 : (awaddr == 'h24 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[1];
    data_length_tap_params_programmed_checker_nxt[2] = (last_sig) ? 0 : (awaddr == 'h28 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[2];
    data_length_tap_params_programmed_checker_nxt[3] = (last_sig) ? 0 : (awaddr == 'h2c && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[3];
    data_length_tap_params_programmed_checker_nxt[4] = (last_sig) ? 0 : (awaddr == 'h30 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[4];
    data_length_tap_params_programmed_checker_nxt[5] = (last_sig) ? 0 : (awaddr == 'h34 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[5];
    data_length_tap_params_programmed_checker_nxt[6] = (last_sig) ? 0 : (awaddr == 'h38 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[6];
    data_length_tap_params_programmed_checker_nxt[7] = (last_sig) ? 0 : (awaddr == 'h3c && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[7];
    data_length_tap_params_programmed_checker_nxt[8] = (last_sig) ? 0 : (awaddr == 'h40 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[8];
    data_length_tap_params_programmed_checker_nxt[9] = (last_sig) ? 0 : (awaddr == 'h44 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[9];
    data_length_tap_params_programmed_checker_nxt[10] = (last_sig) ? 0 : (awaddr == 'h48 && awvalid && awready) ? 1 : data_length_tap_params_programmed_checker[10];

end

always@(posedge axis_clk or negedge axis_rst_n) begin
    for (i=0;i<12;i=i+1) begin
        if (!axis_rst_n) data_length_tap_params_programmed_checker[i] <= 0;
        else             data_length_tap_params_programmed_checker[i] <= data_length_tap_params_programmed_checker_nxt[i];
    end
end

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



// // FIR engine
// reg [(pDATA_WIDTH-1) : 0] Xn, Yn, Hn, Yn_nxt;

// always@(*) begin
//     Xn = data_Do;
//     Hn = tap_Do;
//     if (state_FIR == S_FIR_SSIN)      Yn_nxt = 0; // renew yn to start a new calculation
//     else if (state_FIR == S_FIR_CALC) Yn_nxt = Yn + Xn * Hn; // do conv
//     else                              Yn_nxt = Yn;
// end

// always@(posedge axis_clk or negedge axis_rst_n) begin
//     if (!axis_rst_n) begin
//         Yn <= 0;
//     end else begin
//         Yn <= Yn_nxt;
//     end
// end
// // FIR state machine
// reg [4:0] state_FIR, state_FIR_nxt;

// always@(*) begin
//     case (state_FIR)
//         S_FIR_IDLE:
//         S_FIR_CALC:
//     endcase
// end

// FIR fsm
// reference: https://github.com/holyuming/Verilog-FIR/tree/main
always @(*) begin
    case (state_FIR)
        S_FIR_IDLE:   state_FIR_nxt = (ap_start) ?  S_DATA_RST : S_FIR_IDLE;
        // FIR_IDLE:   state_FIR_nxt = (awaddr == 'h00 && wready && wvalid && wdata[0] == 1) ?  DATA_RST : FIR_IDLE;
        S_DATA_RST:   state_FIR_nxt = (dataRam_rst_cnt == 10) ? S_FIR_WAIT : S_DATA_RST;                        // dataRam reset state
        S_FIR_WAIT:   state_FIR_nxt = (ss_tvalid) ? S_FIR_SIN : S_FIR_WAIT;                               // wait for Stream-In data
        S_FIR_SIN:    state_FIR_nxt = (ss_tready) ? S_FIR_STOR : S_FIR_SIN;                               // write the Stream-In data to dataRAM
        S_FIR_STOR:   state_FIR_nxt = S_FIR_S0;                                                               // wait 1 cycle to let data store into dataRAM
        S_FIR_S0:     state_FIR_nxt = S_FIR_S1;                                                               // 11 states for calculating FIR (n == 11)
        S_FIR_S1:     state_FIR_nxt = S_FIR_S2;
        S_FIR_S2:     state_FIR_nxt = S_FIR_S3;
        S_FIR_S3:     state_FIR_nxt = S_FIR_S4;
        S_FIR_S4:     state_FIR_nxt = S_FIR_S5;
        S_FIR_S5:     state_FIR_nxt = S_FIR_S6;
        S_FIR_S6:     state_FIR_nxt = S_FIR_S7;
        S_FIR_S7:     state_FIR_nxt = S_FIR_S8;
        S_FIR_S8:     state_FIR_nxt = S_FIR_S9;
        S_FIR_S9:     state_FIR_nxt = S_FIR_SA;
        S_FIR_SA:     state_FIR_nxt = S_FIR_OUT;
        S_FIR_OUT: begin
            if      (last_sig)  state_FIR_nxt = S_FIR_IDLE; // reset
            else if (sm_tready) state_FIR_nxt = S_FIR_WAIT; // wait for next Stream-In data
            else                state_FIR_nxt = S_FIR_OUT;  // wait for handshakes
        end
        default: state_FIR_nxt = state_FIR;
    endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)    state_FIR   <= S_FIR_IDLE;
    else                state_FIR   <= state_FIR_nxt;
end


// Get the Xn, Hn from dataRAM & tapRAM
always @(*) begin
    Xn = data_Do;
    Hn = tap_Do;
    if      (state_FIR == S_FIR_SIN) Yn_nxt = 0; // reset accumulator
    else if (S_FIR_S0 <= state_FIR_nxt && state_FIR_nxt <= S_FIR_SA) Yn_nxt = Yn + (Xn * Hn); // accumulate Xn * Hn
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