LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.math_utils.ALL;
ENTITY instr_decoder IS
    GENERIC (
        NUM_REGS_16b : INTEGER := 8;
        NUM_REGS_4b : INTEGER := 8;
        DATA_WIDTH : INTEGER := 16;
        PE_COUNT : INTEGER := 4;
        BASE_ADDR_WIDTH : INTEGER := 20;
        BASE_ADDR_COUNT : INTEGER := 2
    );
    PORT (
        clk_i : IN STD_LOGIC;
        rstn_i : IN STD_LOGIC;
        decode_ena : IN STD_LOGIC;
        cur_instr_in : IN STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
        riscv_data_in : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        riscv_ena : IN STD_LOGIC;
        riscv_rd : IN STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
        ba_ret_in : IN STD_LOGIC_VECTOR(2 * BASE_ADDR_WIDTH - 1 DOWNTO 0);
        ba_incr_in : IN STD_LOGIC_VECTOR(2 * 16 - 1 DOWNTO 0);
        ba_ret_ena_in : IN STD_LOGIC;
        ba_incr_ena_in : IN STD_LOGIC;
        ba_ret_out : OUT STD_LOGIC_VECTOR(2 * BASE_ADDR_WIDTH - 1 DOWNTO 0);
        ba_incr_out : OUT STD_LOGIC_VECTOR(2 * 16 - 1 DOWNTO 0);
        ba_ret_ena_out : OUT STD_LOGIC;
        ba_incr_ena_out : OUT STD_LOGIC;
        ba_out_ena : OUT STD_LOGIC;
        ba_out_idx : OUT STD_LOGIC_VECTOR(log2(BASE_ADDR_COUNT) - 1 DOWNTO 0);
        ba_add_idx : OUT STD_LOGIC_VECTOR(log2(BASE_ADDR_COUNT) - 1 DOWNTO 0);
        ba_add_ena : OUT STD_LOGIC;
        ba_add_dis_pe : OUT STD_LOGIC;
        rel_addr_out : OUT STD_LOGIC_VECTOR(5 - 1 DOWNTO 0);
        riscv_data_out : OUT STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        rs1_addr : OUT STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
        rs2_addr : OUT STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
        rd_addr : OUT STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
        arith_sel : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
        rd_in_sel : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
        pe_ena : OUT STD_LOGIC;
        ce : OUT STD_LOGIC;
        sram_ena : OUT STD_LOGIC;
        sram_rw : OUT STD_LOGIC;
        stall_rq : OUT STD_LOGIC
    );
END ENTITY;
ARCHITECTURE rtl OF instr_decoder IS
    TYPE state_t IS (single_cycle_instr, multi_cycle_instr);
    SIGNAL state : state_t;
    SIGNAL state_next : state_t;

    SIGNAL rel_sram_addr : STD_LOGIC_VECTOR(5 - 1 DOWNTO 0);

    SIGNAL ba_out_ena_next : STD_LOGIC;
    SIGNAL ba_add_ena_next : STD_LOGIC;
    SIGNAL ba_out_idx_next : STD_LOGIC_VECTOR(log2(BASE_ADDR_COUNT) - 1 DOWNTO 0);
    SIGNAL ba_add_idx_next : STD_LOGIC_VECTOR(log2(BASE_ADDR_COUNT) - 1 DOWNTO 0);

    SIGNAL pe_ena_next : STD_LOGIC;
    SIGNAL ba_add_dis_pe_next : STD_LOGIC;

    SIGNAL riscv_data_out_reg_next : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL rs1_addr_reg_next : STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
    SIGNAL rs2_addr_reg_next : STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
    SIGNAL rd_addr_reg_next : STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
    SIGNAL arith_sel_reg_next : STD_LOGIC_VECTOR(3 DOWNTO 0);
    SIGNAL rd_in_sel_reg_next : STD_LOGIC_VECTOR(1 DOWNTO 0);
    SIGNAL ce_reg_next : STD_LOGIC;

    SIGNAL sram_ena_reg_next : STD_LOGIC;
    SIGNAL sram_rw_reg_next : STD_LOGIC;

BEGIN
    riscv_data_out_reg_next <= riscv_data_in;

    -- Determine control of BAIncr instruction
    p_ba_idx : PROCESS (cur_instr_in, decode_ena)
    BEGIN
        IF (cur_instr_in(3 DOWNTO 0) = "0010") THEN
            ba_add_idx_next <= cur_instr_in(log2(BASE_ADDR_COUNT) + 4 - 1 DOWNTO 4);
            ba_add_ena_next <= decode_ena;
            ba_add_dis_pe_next <= cur_instr_in(11);
        ELSE
            ba_add_idx_next <= (OTHERS => '0');
            ba_add_ena_next <= '0';
            ba_add_dis_pe_next <= '0';
        END IF;
    END PROCESS;

    -- Re-enable all PEs 
    p_pe_ena : PROCESS (cur_instr_in, decode_ena)
    BEGIN
        IF (cur_instr_in(4 DOWNTO 0) = "10000" AND decode_ena = '1') THEN
            pe_ena_next <= '1';
        ELSE
            pe_ena_next <= '0';
        END IF;
    END PROCESS;

    -- Enable PEs
    p_ce : PROCESS (cur_instr_in, state, decode_ena, riscv_ena)
    BEGIN
        IF (state = single_cycle_instr AND cur_instr_in(2 DOWNTO 0) = "100") THEN
            ce_reg_next <= riscv_ena;
        ELSIF cur_instr_in(3 DOWNTO 0) = "0000" OR cur_instr_in(3 DOWNTO 0) = "0010" THEN
            ce_reg_next <= riscv_ena;
        ELSE
            ce_reg_next <= decode_ena OR riscv_ena;
        END IF;
    END PROCESS;

    -- Determine SRAM write 
    p_sram_rw : PROCESS (cur_instr_in)
    BEGIN
        IF (cur_instr_in(3 DOWNTO 0) = "1100") THEN
            sram_rw_reg_next <= '1';
        ELSE
            sram_rw_reg_next <= '0';
        END IF;
    END PROCESS;

    -- Determine SRAM ena
    p_sram_ena : PROCESS (cur_instr_in, state, decode_ena)
    BEGIN
        IF (state = single_cycle_instr) THEN
            IF (cur_instr_in(2 DOWNTO 0) = "100") THEN
                sram_ena_reg_next <= decode_ena;
            ELSE
                sram_ena_reg_next <= '0';
            END IF;
        ELSE
            sram_ena_reg_next <= '0';
        END IF;
    END PROCESS;

    -- Determine base address index
    p_ba_out_idx : PROCESS (cur_instr_in)
    BEGIN
        IF cur_instr_in(2 DOWNTO 0) = "100" THEN
            ba_out_idx_next <= cur_instr_in(8 DOWNTO 8);
            ba_out_ena_next <= '1';
        ELSE
            ba_out_idx_next <= (OTHERS => '0');
            ba_out_ena_next <= '0';
        END IF;
    END PROCESS;

    -- Determine relative SRAM address
    p_rel_sram_addr : PROCESS (cur_instr_in)
    BEGIN
        IF cur_instr_in(3 DOWNTO 0) = "1100" THEN
            rel_sram_addr(5 - 1 DOWNTO 0) <= cur_instr_in(11) & cur_instr_in(8 - 1 DOWNTO 4);
        ELSIF cur_instr_in(3 DOWNTO 0) = "0100" THEN
            rel_sram_addr(5 - 1 DOWNTO 0) <= cur_instr_in(16 - 1 DOWNTO 11);
        ELSE
            rel_sram_addr(5 - 1 DOWNTO 0) <= (OTHERS => '0');
        END IF;
    END PROCESS;

    -- Determine RS1 address
    p_rs1_addr : PROCESS (cur_instr_in)
    BEGIN
        IF cur_instr_in(0) = '1' THEN
            rs1_addr_reg_next <= cur_instr_in(12 - 1 DOWNTO 8);
        ELSE
            rs1_addr_reg_next <= (OTHERS => '0');
        END IF;
    END PROCESS;

    -- Determine RS2 address
    p_rs2_addr : PROCESS (cur_instr_in)
    BEGIN
        IF (cur_instr_in(0) = '1' OR cur_instr_in(3 DOWNTO 0) = "1100" OR cur_instr_in(3 DOWNTO 0) = "0010") THEN
            rs2_addr_reg_next <= cur_instr_in(16 - 1 DOWNTO 12);
        ELSE
            rs2_addr_reg_next <= (OTHERS => '0');
        END IF;
    END PROCESS;

    -- Determine RD address
    p_rd_addr : PROCESS (cur_instr_in, decode_ena, riscv_rd, riscv_ena)
    BEGIN
        IF (decode_ena = '1') THEN
            IF (cur_instr_in(0) = '1' OR cur_instr_in(3 DOWNTO 0) = "0100") THEN
                rd_addr_reg_next <= cur_instr_in(8 - 1 DOWNTO 4);
            ELSE
                rd_addr_reg_next <= (OTHERS => '0');
            END IF;
        ELSIF (riscv_ena = '1') THEN
            rd_addr_reg_next <= riscv_rd;
        ELSE
            rd_addr_reg_next <= (OTHERS => '0');
        END IF;
    END PROCESS;

    -- Select arithmetic unit
    p_arith_sel : PROCESS (cur_instr_in)
    BEGIN
        CASE (cur_instr_in(3 DOWNTO 0)) IS
            WHEN "0001" =>
                arith_sel_reg_next <= "0001";
            WHEN "0011" =>
                arith_sel_reg_next <= "1000";
            WHEN "0111" =>
                arith_sel_reg_next <= "0010";
            WHEN "1001" =>
                arith_sel_reg_next <= "0100";
            WHEN OTHERS =>
                arith_sel_reg_next <= "0000";
        END CASE;
    END PROCESS;

    -- Select source of input of register file
    p_rd_in_sel : PROCESS (cur_instr_in, decode_ena, riscv_ena)
    BEGIN
        IF (cur_instr_in(0) = '1' AND decode_ena = '1') THEN
            rd_in_sel_reg_next <= "00";
        ELSIF (decode_ena = '0' AND riscv_ena = '1') THEN
            rd_in_sel_reg_next <= "01";
        ELSE
            rd_in_sel_reg_next <= "10";
        END IF;
    END PROCESS;

    -- Determine next state
    p_state : PROCESS (state, decode_ena, cur_instr_in)
    BEGIN
        IF (decode_ena = '1') THEN
            IF (cur_instr_in(3 DOWNTO 0) = "0100" AND state = single_cycle_instr) THEN
                state_next <= multi_cycle_instr;
            ELSE
                state_next <= single_cycle_instr;
            END IF;
        ELSE
            state_next <= single_cycle_instr;
        END IF;
    END PROCESS;

    -- Stall the loop controller for multi-cycle instructions
    p_stall_rq : PROCESS (state, cur_instr_in, decode_ena)
    BEGIN
        IF (state = single_cycle_instr AND cur_instr_in(3 DOWNTO 0) = "0100" AND decode_ena = '1') THEN
            stall_rq <= '1';
        ELSE
            stall_rq <= '0';
        END IF;
    END PROCESS;

    -- Pipeline all output signals to SRAM and PEs and AGUs
    p_reg : PROCESS (clk_i, rstn_i)
    BEGIN
        IF rstn_i = '0' THEN
            state <= single_cycle_instr;
            riscv_data_out <= (OTHERS => '0');
            rs1_addr <= (OTHERS => '0');
            rs2_addr <= (OTHERS => '0');
            rd_addr <= (OTHERS => '0');
            arith_sel <= (OTHERS => '0');
            rd_in_sel <= (OTHERS => '0');
            ce <= '0';
            sram_ena <= '0';
            sram_rw <= '0';
            ba_add_dis_pe <= '0';

            ba_ret_out <= (OTHERS => '0');
            ba_incr_out <= (OTHERS => '0');
            ba_ret_ena_out <= '0';
            ba_incr_ena_out <= '0';
            rel_addr_out <= (OTHERS => '0');
            ba_out_idx <= (OTHERS => '0');
            ba_add_idx <= (OTHERS => '0');
            ba_out_ena <= '0';
            ba_add_ena <= '0';

        ELSIF rising_edge(clk_i) THEN
            state <= state_next;
            ce <= ce_reg_next;
            sram_ena <= sram_ena_reg_next;
            riscv_data_out <= riscv_data_out_reg_next;
            rs1_addr <= rs1_addr_reg_next;
            rs2_addr <= rs2_addr_reg_next;
            rd_addr <= rd_addr_reg_next;
            arith_sel <= arith_sel_reg_next;
            rd_in_sel <= rd_in_sel_reg_next;
            sram_rw <= sram_rw_reg_next;

            ba_ret_out <= ba_ret_in;
            ba_incr_out <= ba_incr_in;
            ba_ret_ena_out <= ba_ret_ena_in;
            ba_incr_ena_out <= ba_incr_ena_in;
            rel_addr_out <= rel_sram_addr;
            ba_out_idx <= ba_out_idx_next;
            ba_add_idx <= ba_add_idx_next;
            ba_out_ena <= ba_out_ena_next;
            ba_add_ena <= ba_add_ena_next;
            ba_add_dis_pe <= ba_add_dis_pe_next;

            pe_ena <= pe_ena_next;

        END IF;
    END PROCESS;

END ARCHITECTURE;