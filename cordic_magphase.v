// CORDIC Magnitude and Phase Calculator
// Author: William Anthony
// Date: 2 January 2025
// Affiliation: Third Year Electrical Engineering Student at Bandung Institute of Technology
//
// Iterative CORDIC in vectoring mode with correct quadrant handling
// Fixed-point angle: radians scaled by 2^28 (Q3.28-like)
// Magnitude output: scaled by 2^OUTPUT_FRAC_BITS (default Q18.14)
// One iteration per cycle, FSM based, synthesizable

module cordic_magphase #(
    parameter integer INPUT_WIDTH      = 16,
    parameter integer INT_WIDTH        = 32,
    parameter integer ITERATIONS       = 32,
    parameter integer GAIN_FRAC_BITS   = 28,
    parameter integer OUTPUT_FRAC_BITS = 14,
    // Pipeline depth for magnitude output (shift-register). Set to 1 for no extra latency.
    parameter integer MAG_PIPELINE     = 2,
    // Input pipeline depth (number of register stages before CORDIC starts)
    parameter integer INPUT_PIPELINE   = 1
) (
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          start,
    input  wire signed [INPUT_WIDTH-1:0] x_in,
    input  wire signed [INPUT_WIDTH-1:0] y_in,
    output reg                           busy,
    output reg                           done,
    output reg  signed [INT_WIDTH-1:0]   magnitude,
    output reg  signed [INT_WIDTH-1:0]   phase
);

    // FSM states (expanded to 3 bits to add LOAD_INPUT)
    localparam IDLE         = 3'd0;
    localparam LOAD_INPUT   = 3'd1;
    localparam ITERATE      = 3'd2;
    localparam POST_PROCESS = 3'd3;
    localparam DONE_STATE   = 3'd4;

    reg [2:0] state, state_next;

    // Datapath registers
    reg signed [INT_WIDTH-1:0] x_reg, y_reg, x_next, y_next;
    reg signed [INT_WIDTH-1:0] z_reg, z_next;
    reg [5:0]                  iter_cnt, iter_cnt_next;

    // Phase correction flag based on initial x sign
    reg signed [1:0] pi_correction, pi_correction_next; // -1,0,+1 multiply by pi

    // Constants
    localparam [31:0] PI_CONST32    = 32'h3243F6A8; // pi * 2^28
    localparam [31:0] TWOPI_CONST32 = 32'h6487ED51; // 2*pi * 2^28
    // K_INV for 32 iterations, Q28: optimized for magnitude accuracy ~1414
    localparam [31:0] K_INV32       = 32'd162600000;
    localparam signed [INT_WIDTH-1:0] PI_CONST    = {{(INT_WIDTH-32){PI_CONST32[31]}}, PI_CONST32};
    localparam signed [INT_WIDTH-1:0] TWOPI_CONST = {{(INT_WIDTH-32){TWOPI_CONST32[31]}}, TWOPI_CONST32};
    localparam signed [INT_WIDTH-1:0] K_INV       = {{(INT_WIDTH-32){K_INV32[31]}}, K_INV32};

    // atan lookup (returns signed 32-bit angle scaled by 2^28)
    function signed [31:0] atan_lut;
        input [4:0] idx;
        begin
            case (idx)
                4'd0:  atan_lut = 32'sh0C90FDAA;
                4'd1:  atan_lut = 32'sh076B19C1;
                4'd2:  atan_lut = 32'sh03EB6EBF;
                4'd3:  atan_lut = 32'sh01FD5BA9;
                4'd4:  atan_lut = 32'sh00FFAAE0;
                4'd5:  atan_lut = 32'sh007FF55B;
                4'd6:  atan_lut = 32'sh003FFEA8;
                4'd7:  atan_lut = 32'sh001FFFDA;
                4'd8:  atan_lut = 32'sh000FFFEF;
                4'd9:  atan_lut = 32'sh0007FFFF;
                4'd10: atan_lut = 32'sh0003FFFF;
                4'd11: atan_lut = 32'sh0001FFFF;
                4'd12: atan_lut = 32'sh0000FFFF;
                4'd13: atan_lut = 32'sh00007FFF;
                4'd14: atan_lut = 32'sh00003FFF;
                4'd15: atan_lut = 32'sh00001FFF; // atan(2^-15) * 2^28
                5'd16: atan_lut = 32'sh00000FFF; // atan(2^-16) * 2^28
                5'd17: atan_lut = 32'sh000007FF; // atan(2^-17) * 2^28
                5'd18: atan_lut = 32'sh000003FF; // atan(2^-18) * 2^28
                5'd19: atan_lut = 32'sh000001FF; // atan(2^-19) * 2^28
                5'd20: atan_lut = 32'sh000000FF; // atan(2^-20) * 2^28
                5'd21: atan_lut = 32'sh0000007F; // atan(2^-21) * 2^28
                5'd22: atan_lut = 32'sh0000003F; // atan(2^-22) * 2^28
                5'd23: atan_lut = 32'sh0000001F; // atan(2^-23) * 2^28
                5'd24: atan_lut = 32'sh0000000F; // atan(2^-24) * 2^28
                5'd25: atan_lut = 32'sh00000007; // atan(2^-25) * 2^28
                5'd26: atan_lut = 32'sh00000003; // atan(2^-26) * 2^28
                5'd27: atan_lut = 32'sh00000001; // atan(2^-27) * 2^28
                default: atan_lut = 32'sh00000000; // idx >= 28, negligible
            endcase
        end
    endfunction

    // Magnitude gain compensation helper
    wire signed [47:0] mag_prod = x_reg * K_INV; // Q(INT+GAIN_FRAC_BITS)
    wire signed [INT_WIDTH-1:0] mag_corr = mag_prod >>> GAIN_FRAC_BITS;
    reg  signed [INT_WIDTH-1:0] phase_tmp;
    // Pipeline storage for magnitude and phase outputs
    integer i;
    reg signed [INT_WIDTH-1:0] mag_pipe [0:MAG_PIPELINE-1];
    reg signed [INT_WIDTH-1:0] phase_pipe [0:MAG_PIPELINE-1];
    reg [MAG_PIPELINE-1:0] mag_valid_sr;
    // Input pipeline registers
    reg signed [INT_WIDTH-1:0] in_pipe_x [0:INPUT_PIPELINE-1];
    reg signed [INT_WIDTH-1:0] in_pipe_y [0:INPUT_PIPELINE-1];
    reg [INPUT_PIPELINE-1:0] in_valid_sr;
    reg [INPUT_PIPELINE-1:0] in_valid_next;

    // Sequential
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            busy      <= 1'b0;
            done      <= 1'b0;
            x_reg     <= 0;
            y_reg     <= 0;
            z_reg     <= 0;
            iter_cnt  <= 0;
            magnitude <= 0;
            phase     <= 0;
            pi_correction <= 2'sd0;
            mag_valid_sr <= {MAG_PIPELINE{1'b0}};
            in_valid_sr <= {INPUT_PIPELINE{1'b0}};
            for (i = 0; i < MAG_PIPELINE; i = i + 1) begin
                mag_pipe[i] <= 0;
                phase_pipe[i] <= 0;
            end
            for (i = 0; i < INPUT_PIPELINE; i = i + 1) begin
                in_pipe_x[i] <= 0;
                in_pipe_y[i] <= 0;
            end
        end else begin
            state    <= state_next;
            x_reg    <= x_next;
            y_reg    <= y_next;
            z_reg    <= z_next;
            iter_cnt <= iter_cnt_next;
            pi_correction <= pi_correction_next;
            // Input pipeline shifting: capture on start when in IDLE, otherwise shift
            // compute next in_valid shift-register safely for any INPUT_PIPELINE
            in_valid_next[0] = (state == IDLE && start) ? 1'b1 : 1'b0;
            for (i = 1; i < INPUT_PIPELINE; i = i + 1)
                in_valid_next[i] = in_valid_sr[i-1];
            in_valid_sr <= in_valid_next;

            in_pipe_x[0] <= (state == IDLE && start) ? {{(INT_WIDTH-INPUT_WIDTH){x_in[INPUT_WIDTH-1]}}, x_in} : 0;
            in_pipe_y[0] <= (state == IDLE && start) ? {{(INT_WIDTH-INPUT_WIDTH){y_in[INPUT_WIDTH-1]}}, y_in} : 0;
            for (i = 1; i < INPUT_PIPELINE; i = i + 1) begin
                in_pipe_x[i] <= in_pipe_x[i-1];
                in_pipe_y[i] <= in_pipe_y[i-1];
            end
            // Push scaled results into pipeline when final result ready
            if (state_next == DONE_STATE) begin
                // Scale magnitude to OUTPUT_FRAC_BITS (shift left to add fractional bits)
                // Load directly into pipeline first stage to avoid extra cycle of delay
                mag_pipe[0] <= ((mag_corr < 0) ? -mag_corr : mag_corr) << OUTPUT_FRAC_BITS;
                case (pi_correction)
                    -2'sd1: phase_tmp = z_next - PI_CONST;
                    2'sd0:  phase_tmp = z_next;
                    default: phase_tmp = z_next + PI_CONST;
                endcase
                if (phase_tmp > PI_CONST)
                    phase_tmp = phase_tmp - TWOPI_CONST;
                else if (phase_tmp < -PI_CONST)
                    phase_tmp = phase_tmp + TWOPI_CONST;
                // (phase_pipe[0] already assigned below)
                phase_pipe[0] <= phase_tmp;
                mag_valid_sr <= {mag_valid_sr[MAG_PIPELINE-2:0], 1'b1};
            end else begin
                // no new result this cycle
                mag_pipe[0] <= 0;
                phase_pipe[0] <= 0;
                mag_valid_sr <= {mag_valid_sr[MAG_PIPELINE-2:0], 1'b0};
            end

            // Shift remaining pipeline stages
            for (i = 1; i < MAG_PIPELINE; i = i + 1) begin
                mag_pipe[i] <= mag_pipe[i-1];
                phase_pipe[i] <= phase_pipe[i-1];
            end

            // Output from last stage
            magnitude <= mag_pipe[MAG_PIPELINE-1];
            phase <= phase_pipe[MAG_PIPELINE-1];

            // busy when processing or pipeline not empty; done when last stage valid
            busy <= (state_next != IDLE) || (|mag_valid_sr != 0);
            done <= mag_valid_sr[MAG_PIPELINE-1];
        end
    end

    // Combinational
    always @* begin
        // defaults
        state_next          = state;
        x_next              = x_reg;
        y_next              = y_reg;
        z_next              = z_reg;
        iter_cnt_next       = iter_cnt;
        pi_correction_next  = pi_correction;

        case (state)
            IDLE: begin
                if (start) begin
                        // start -> capture inputs into input pipeline
                        state_next = LOAD_INPUT;
                    end
            end

                LOAD_INPUT: begin
                    // wait until input pipeline valid shifts through
                    if (in_valid_sr[INPUT_PIPELINE-1]) begin
                        x_next = in_pipe_x[INPUT_PIPELINE-1];
                        y_next = in_pipe_y[INPUT_PIPELINE-1];
                        // If x is negative, rotate vector by 180 deg to bring into right half-plane
                        if (x_next < 0) begin
                            x_next = -x_next;
                            y_next = -y_next;
                            pi_correction_next = (y_next >= 0) ? 2'sd1 : -2'sd1;
                        end else begin
                            pi_correction_next = 2'sd0;
                        end
                        z_next        = 0;
                        iter_cnt_next = 0;
                        state_next    = ITERATE;
                    end else begin
                        state_next = LOAD_INPUT;
                    end
                end

            ITERATE: begin
                if (iter_cnt < ITERATIONS) begin
                    if (y_reg >= 0) begin
                        // Rotate clockwise to drive y toward zero (vectoring mode)
                        x_next = x_reg + (y_reg >>> iter_cnt);
                        y_next = y_reg - (x_reg >>> iter_cnt);
                        z_next = z_reg + atan_lut(iter_cnt[4:0]);
                    end else begin
                        // Rotate counter-clockwise when y is negative
                        x_next = x_reg - (y_reg >>> iter_cnt);
                        y_next = y_reg + (x_reg >>> iter_cnt);
                        z_next = z_reg - atan_lut(iter_cnt[4:0]);
                    end
                    iter_cnt_next = iter_cnt + 1'b1;
                    if (iter_cnt == ITERATIONS-1)
                        state_next = POST_PROCESS;
                end else begin
                    state_next = POST_PROCESS;
                end
            end

            POST_PROCESS: begin
                // No extra math here; magnitude/phase computed in sequential on transition to DONE
                state_next = DONE_STATE;
            end

            DONE_STATE: begin
                state_next = IDLE;
            end
            default: state_next = IDLE;
        endcase
    end

endmodule
