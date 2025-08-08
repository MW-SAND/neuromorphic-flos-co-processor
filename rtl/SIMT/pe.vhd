LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.MATH_UTILS.ALL;

ENTITY pe IS
    GENERIC (
        NUM_REGS_16b : INTEGER := 8;
        NUM_REGS_4b : INTEGER := 8;
        DATA_WIDTH : INTEGER := 16
    );
    PORT (
        rstn_i : IN STD_LOGIC;
        clk_i : IN STD_LOGIC;
        ce : IN STD_LOGIC;
        sram_in : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        riscv_in : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        rs1_addr : IN STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
        rs2_addr : IN STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
        rd_addr : IN STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
        arith_sel : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
        rd_in_sel : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
        ba_add_ena : IN STD_LOGIC;
        data_out : OUT STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0)
    );
END ENTITY pe;
ARCHITECTURE rtl OF pe IS

    SIGNAL rs1, rs2, rd, arith_out : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL rs1_4b, rs2_4b, rs1_16b, rs2_16b : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);

    SIGNAL max_out, mult_out, add_out, shift_out : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);

    SIGNAL wena_16b, wena_4b : STD_LOGIC;
BEGIN

    data_out <= rs2;

    -- Select register file input
    p_rd : PROCESS (arith_out, riscv_in, sram_in, rd_in_sel)
    BEGIN
        CASE rd_in_sel IS
            WHEN "00" => rd <= arith_out;
            WHEN "01" => rd <= riscv_in;
            WHEN "10" => rd <= sram_in;
            WHEN OTHERS => rd <= (OTHERS => 'X');
        END CASE;
    END PROCESS;

    -- Select arithmetic output
    p_arith_out : PROCESS (max_out, shift_out, add_out, mult_out, arith_sel)
    BEGIN
        CASE arith_sel IS
            WHEN "0001" => arith_out <= add_out;
            WHEN "0010" => arith_out <= max_out;
            WHEN "0100" => arith_out <= shift_out;
            WHEN "1000" => arith_out <= mult_out;
            WHEN OTHERS => arith_out <= add_out;
        END CASE;
    END PROCESS;

    -- Select RS1 and RS2 from both register files
    rs1 <= rs1_16b WHEN rs1_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 1) = '0' ELSE
        rs1_4b;
    rs2 <= rs2_16b WHEN rs2_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 1) = '0' ELSE
        rs2_4b;

    -- Determine write enable signals for both register files
    wena_16b <= ce AND NOT(rd_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 1));
    wena_4b <= ce AND rd_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 1);

    rf_16b : ENTITY work.register_file
        GENERIC MAP(
            REG_DATA_WIDTH => DATA_WIDTH,
            REG_NUM => NUM_REGS_16b,
            DATA_IN_DATA_WIDTH => DATA_WIDTH,
            DATA_OUT_DATA_WIDTH => DATA_WIDTH
        )
        PORT MAP(
            wena => wena_16b,
            reg_A_addr => rs1_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 2 DOWNTO 0),
            reg_B_addr => rs2_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 2 DOWNTO 0),
            waddr => rd_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 2 DOWNTO 0),
            data_in => rd,
            clk_i => clk_i,
            ba_add_ena => ba_add_ena,
            data_out_A => rs1_16b,
            data_out_B => rs2_16b
        );

    rf_4b : ENTITY work.register_file
        GENERIC MAP(
            REG_DATA_WIDTH => 4,
            REG_NUM => NUM_REGS_4b,
            DATA_IN_DATA_WIDTH => DATA_WIDTH,
            DATA_OUT_DATA_WIDTH => DATA_WIDTH
        )
        PORT MAP(
            wena => wena_4b,
            reg_A_addr => rs1_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 2 DOWNTO 0),
            reg_B_addr => rs2_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 2 DOWNTO 0),
            waddr => rd_addr(log2(NUM_REGS_16b + NUM_REGS_4b) - 2 DOWNTO 0),
            data_in => rd,
            clk_i => clk_i,
            ba_add_ena => ba_add_ena,
            data_out_A => rs1_4b,
            data_out_B => rs2_4b
        );

    mul0 : ENTITY work.multiplier
        GENERIC MAP(DATA_WIDTH => DATA_WIDTH)
        PORT MAP(
            op1 => rs1,
            op2 => rs2,
            ena => arith_sel(3),
            prod => mult_out
        );

    max0 : ENTITY work.max
        GENERIC MAP(DATA_WIDTH => DATA_WIDTH)
        PORT MAP(
            op1 => rs1,
            op2 => rs2,
            ena => arith_sel(1),
            max_op => max_out
        );

    bs0 : ENTITY work.barrelshifter
        GENERIC MAP(DATA_WIDTH => DATA_WIDTH)
        PORT MAP(
            op1 => rs1,
            op2 => rs2,
            ena => arith_sel(2),
            op1_shifted => shift_out
        );

    add0 : ENTITY work.adder
        GENERIC MAP(DATA_WIDTH => DATA_WIDTH)
        PORT MAP(
            op1 => rs1,
            op2 => rs2,
            ena => arith_sel(0),
            sum => add_out
        );

END ARCHITECTURE;