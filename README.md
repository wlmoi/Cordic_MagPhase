# CORDIC Magnitude and Phase Calculator

This project implements a CORDIC (COordinate Rotation DIgital Computer) algorithm in Verilog to calculate the magnitude and phase of a given vector. The implementation is optimized for accuracy and performance, making it suitable for FPGA and ASIC designs.

## Features
- **Accurate Magnitude and Phase Calculation**: Supports all quadrants with high precision.
- **Fixed-Point Arithmetic**: Uses Q28 precision for gain compensation and atan lookup.
- **Configurable Parameters**: Easily adjust precision and iterations.
- **Testbench Included**: Comprehensive testbench to validate functionality.

## Files
- `cordic_magphase.v`: Verilog module implementing the CORDIC algorithm.
- `tb_cordic_magphase.v`: Testbench for verifying the functionality of the CORDIC module.

## Simulation Results
The testbench validates the module with the following results:
```
Time 225000 ns | x=1000 y=1000 | magnitude=1414 | phase_deg=44.983403
Time 415000 ns | x=-1000 y=1000 | magnitude=1414 | phase_deg=134.983403
Time 605000 ns | x=-1000 y=-1000 | magnitude=1414 | phase_deg=-135.016597
Time 795000 ns | x=1000 y=-1000 | magnitude=1414 | phase_deg=-45.016597
All tests completed
```

## How to Run
1. Install a Verilog simulator (e.g., Icarus Verilog).
2. Compile and run the testbench:
   ```bash
   iverilog -g2005-sv -o cordic_tb tb_cordic_magphase.v
   vvp cordic_tb
   ```
3. Observe the simulation results in the terminal.

## Applications
- Digital Signal Processing (DSP)
- Robotics and Control Systems
- Communication Systems

## License
This project is open-source and available under the MIT License.