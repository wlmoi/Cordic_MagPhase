`timescale 1ns/1ps

`include "cordic_magphase.v"

module tb_cordic_magphase;
    localparam INPUT_WIDTH = 16;
    localparam INT_WIDTH   = 32;
    localparam ITERATIONS  = 16;

    reg clk;
    reg rst_n;
    reg start;
    reg signed [INPUT_WIDTH-1:0] x_in;
    reg signed [INPUT_WIDTH-1:0] y_in;
    wire busy;
    wire done;
    wire signed [INT_WIDTH-1:0] magnitude;
    wire signed [INT_WIDTH-1:0] phase;

    cordic_magphase #(
        .INPUT_WIDTH(INPUT_WIDTH),
        .INT_WIDTH(INT_WIDTH),
        .ITERATIONS(ITERATIONS)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .x_in(x_in),
        .y_in(y_in),
        .busy(busy),
        .done(done),
        .magnitude(magnitude),
        .phase(phase)
    );

    // Clock
    initial clk = 1'b0;
    always #5 clk = ~clk; // 100 MHz

    // Utility to display results in degrees (phase uses radians scaled by 2^28)
    real phase_deg;
    real magnitude_real;

    task show_result;
        begin
            phase_deg = (phase * 1.0) * 180.0 / 3.141592653589793 / (2.0**28);
            magnitude_real = magnitude;
            $display("Time %0t ns | x=%0d y=%0d | magnitude=%0d | phase_deg=%0f", $time, x_in, y_in, magnitude, phase_deg);
        end
    endtask

    task apply_vector(input signed [INPUT_WIDTH-1:0] x, input signed [INPUT_WIDTH-1:0] y);
        begin
            @(negedge clk);
            x_in <= x;
            y_in <= y;
            start <= 1'b1;
            @(negedge clk);
            start <= 1'b0;
            // wait for done
            wait(done);
            show_result();
            @(negedge clk);
        end
    endtask

    initial begin
        // init
        x_in  = 0;
        y_in  = 0;
        start = 0;
        rst_n = 0;
        repeat (3) @(negedge clk);
                // let done/busy drop before next vector
                @(negedge clk);
        rst_n = 1;

        // Quadrant tests
        apply_vector( 16'sd1000,  16'sd1000); // Q1
        apply_vector(-16'sd1000,  16'sd1000); // Q2 critical
        apply_vector(-16'sd1000, -16'sd1000); // Q3 critical
        apply_vector( 16'sd1000, -16'sd1000); // Q4

        $display("All tests completed");
        #20;
        $finish;
    end
endmodule
