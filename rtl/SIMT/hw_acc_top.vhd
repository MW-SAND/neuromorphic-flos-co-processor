LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY neorv32;
USE neorv32.neorv32_package.ALL;

USE work.math_utils.ALL;
ENTITY hw_acc_top IS
    GENERIC (
        NUM_REGS_16b : INTEGER := 8;
        NUM_REGS_4b : INTEGER := 8;
        DATA_WIDTH : INTEGER := 16;
        BASE_ADDR_WIDTH : INTEGER := 20;
        BASE_ADDR_COUNT : INTEGER := 2;
        NPE_COUNT : INTEGER := 1
    );
    PORT (
        clk_i : IN STD_LOGIC;
        rstn_i : IN STD_LOGIC;
        bus_req_i : IN bus_req_t;
        bus_rsp_o : OUT bus_rsp_t;
        sram_data_in : IN STD_LOGIC_VECTOR(NPE_COUNT * DATA_WIDTH - 1 DOWNTO 0);
        sram_ena : OUT STD_LOGIC;
        sram_pe_ena : OUT STD_LOGIC_VECTOR(NPE_COUNT - 1 DOWNTO 0);
        sram_rw : OUT STD_LOGIC;
        sram_addr : OUT STD_LOGIC_VECTOR(NPE_COUNT * 24 - 1 DOWNTO 0);
        sram_data_out : OUT STD_LOGIC_VECTOR(NPE_COUNT * DATA_WIDTH - 1 DOWNTO 0);
        irq_o : OUT STD_LOGIC
    );
END ENTITY;
ARCHITECTURE structural OF hw_acc_top IS

    COMPONENT loop_buffer IS
        GENERIC (
            NUM_REGS_16b : INTEGER := 8;
            NUM_REGS_4b : INTEGER := 8;
            DATA_WIDTH : INTEGER := 16;
            BASE_ADDR_WIDTH : INTEGER := 20;
            TASK_BUFFER_SIZE : INTEGER := 16;
            TASK_SIZE : INTEGER := 24;
            IRQ_EMPTY_OFFSET : INTEGER := 4;
            NPE_COUNT : INTEGER := 8
        );
        PORT (
            clk_i : IN STD_LOGIC;
            stall_rq : IN STD_LOGIC;
            rstn_i : IN STD_LOGIC;
            bus_req_i : IN bus_req_t;
            agu_ce_in : IN STD_LOGIC_VECTOR(NPE_COUNT - 1 DOWNTO 0);
            bus_rsp_o : OUT bus_rsp_t;
            decode_ena : OUT STD_LOGIC;
            irq_o : OUT STD_LOGIC;
            cur_instr_out : OUT STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
            ba_ret_out : OUT STD_LOGIC_VECTOR(2 * BASE_ADDR_WIDTH - 1 DOWNTO 0);
            ba_incr_out : OUT STD_LOGIC_VECTOR(2 * 16 - 1 DOWNTO 0);
            ba_ret_ena_out : OUT STD_LOGIC;
            ba_incr_ena_out : OUT STD_LOGIC;
            riscv_data : OUT STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
            riscv_rd : OUT STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
            riscv_ena : OUT STD_LOGIC
        );
    END COMPONENT;

    COMPONENT npe IS
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
    END COMPONENT;

    COMPONENT instr_decoder IS
        GENERIC (
            NUM_REGS_16b : INTEGER := 8;
            NUM_REGS_4b : INTEGER := 8;
            DATA_WIDTH : INTEGER := 16;
            NPE_COUNT : INTEGER := 4;
            BASE_ADDR_WIDTH : INTEGER := 20
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
    END COMPONENT;
    COMPONENT agu IS
        GENERIC (
            DATA_WIDTH : INTEGER := 16;
            BASE_ADDR_WIDTH : INTEGER := 20;
            BASE_ADDR_COUNT : INTEGER := 2;
            NPE_COUNT : INTEGER := 8
        );
        PORT (
            clk : IN STD_LOGIC;
            rstn_i : IN STD_LOGIC;
            ba_ret : IN STD_LOGIC_VECTOR(BASE_ADDR_WIDTH * BASE_ADDR_COUNT - 1 DOWNTO 0);
            ba_ret_ena : IN STD_LOGIC;
            ba_incr : IN STD_LOGIC_VECTOR(16 * BASE_ADDR_COUNT - 1 DOWNTO 0);
            ba_incr_ena : IN STD_LOGIC;
            ba_add_idx : IN STD_LOGIC_VECTOR(log2(BASE_ADDR_COUNT) - 1 DOWNTO 0);
            ba_add_val : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
            ba_add_ena : IN STD_LOGIC;
            ba_add_dis_pe : IN STD_LOGIC;
            ena_pe : IN STD_LOGIC;
            rel_addr : IN STD_LOGIC_VECTOR(5 - 1 DOWNTO 0);
            ba_out_idx : IN STD_LOGIC_VECTOR(log2(BASE_ADDR_COUNT) - 1 DOWNTO 0);
            ba_out_ena : IN STD_LOGIC;
            addr_out : OUT STD_LOGIC_VECTOR(24 - 1 DOWNTO 0);
            pe_ena_out : OUT STD_LOGIC
        );
    END COMPONENT;

    SIGNAL npe_data_out : STD_LOGIC_VECTOR(NPE_COUNT * DATA_WIDTH - 1 DOWNTO 0);

    SIGNAL stall_rq : STD_LOGIC;
    SIGNAL decode_ena : STD_LOGIC;
    SIGNAL cur_instr : STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);

    SIGNAL ba_ret_lb : STD_LOGIC_VECTOR(2 * BASE_ADDR_WIDTH - 1 DOWNTO 0);
    SIGNAL ba_incr_lb : STD_LOGIC_VECTOR(2 * 16 - 1 DOWNTO 0);
    SIGNAL ba_ret_ena_lb : STD_LOGIC;
    SIGNAL ba_incr_ena_lb : STD_LOGIC;

    SIGNAL ba_ret_id : STD_LOGIC_VECTOR(2 * BASE_ADDR_WIDTH - 1 DOWNTO 0);
    SIGNAL ba_incr_id : STD_LOGIC_VECTOR(2 * 16 - 1 DOWNTO 0);
    SIGNAL ba_ret_ena_id : STD_LOGIC;
    SIGNAL ba_incr_ena_id : STD_LOGIC;
    SIGNAL ba_out_ena : STD_LOGIC;
    SIGNAL ba_out_idx : STD_LOGIC_VECTOR(log2(BASE_ADDR_COUNT) - 1 DOWNTO 0);
    SIGNAL ba_add_idx : STD_LOGIC_VECTOR(log2(BASE_ADDR_COUNT) - 1 DOWNTO 0);
    SIGNAL ba_add_ena : STD_LOGIC;

    SIGNAL ba_add_dis_pe : STD_LOGIC;

    SIGNAL riscv_data, riscv_data_npe : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL riscv_rd : STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
    SIGNAL riscv_ena : STD_LOGIC;
    SIGNAL pe_ena : STD_LOGIC;

    SIGNAL rs1_addr, rs2_addr, rd_addr : STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
    SIGNAL arith_sel : STD_LOGIC_VECTOR(3 DOWNTO 0);
    SIGNAL rd_in_sel : STD_LOGIC_VECTOR(1 DOWNTO 0);
    SIGNAL ce : STD_LOGIC;

    SIGNAL npe_ce : STD_LOGIC_VECTOR(NPE_COUNT - 1 DOWNTO 0);
    SIGNAL agu_ce_out : STD_LOGIC_VECTOR(NPE_COUNT - 1 DOWNTO 0);

    SIGNAL rel_addr : STD_LOGIC_VECTOR(5 - 1 DOWNTO 0);

BEGIN

    -- Instantiate Loop Controller
    loop_buffer_0 : loop_buffer
    GENERIC MAP
    (
        NUM_REGS_16b => NUM_REGS_16b,
        NUM_REGS_4b => NUM_REGS_4b,
        DATA_WIDTH => DATA_WIDTH,
        BASE_ADDR_WIDTH => BASE_ADDR_WIDTH,
        NPE_COUNT => NPE_COUNT
    )
    PORT MAP
    (
        clk_i => clk_i,
        rstn_i => rstn_i,
        stall_rq => stall_rq,
        bus_req_i => bus_req_i,
        agu_ce_in => agu_ce_out,
        bus_rsp_o => bus_rsp_o,
        decode_ena => decode_ena,
        irq_o => irq_o,
        cur_instr_out => cur_instr,
        ba_ret_out => ba_ret_lb,
        ba_incr_out => ba_incr_lb,
        ba_ret_ena_out => ba_ret_ena_lb,
        ba_incr_ena_out => ba_incr_ena_lb,
        riscv_data => riscv_data,
        riscv_rd => riscv_rd,
        riscv_ena => riscv_ena
    );

    -- Instantiate Instruction Decoder
    decoder_0 : instr_decoder
    GENERIC MAP
    (
        NUM_REGS_16b => NUM_REGS_16b,
        NUM_REGS_4b => NUM_REGS_4b,
        DATA_WIDTH => DATA_WIDTH,
        NPE_COUNT => NPE_COUNT,
        BASE_ADDR_WIDTH => BASE_ADDR_WIDTH
    )
    PORT MAP
    (
        clk_i => clk_i,
        rstn_i => rstn_i,
        decode_ena => decode_ena,
        cur_instr_in => cur_instr,
        riscv_data_in => riscv_data,
        riscv_rd => riscv_rd,
        riscv_ena => riscv_ena,
        ba_ret_in => ba_ret_lb,
        ba_incr_in => ba_incr_lb,
        ba_ret_ena_in => ba_ret_ena_lb,
        ba_incr_ena_in => ba_incr_ena_lb,
        ba_ret_out => ba_ret_id,
        ba_incr_out => ba_incr_id,
        ba_ret_ena_out => ba_ret_ena_id,
        ba_incr_ena_out => ba_incr_ena_id,
        ba_out_ena => ba_out_ena,
        ba_out_idx => ba_out_idx,
        ba_add_idx => ba_add_idx,
        ba_add_ena => ba_add_ena,
        ba_add_dis_pe => ba_add_dis_pe,
        rel_addr_out => rel_addr,
        riscv_data_out => riscv_data_npe,
        sram_ena => sram_ena,
        sram_rw => sram_rw,
        stall_rq => stall_rq,
        rs1_addr => rs1_addr,
        rs2_addr => rs2_addr,
        rd_in_sel => rd_in_sel,
        rd_addr => rd_addr,
        arith_sel => arith_sel,
        pe_ena => pe_ena,
        ce => ce
    );

    -- Instantiate array of PE and AGU
    gen_npe : FOR i IN 0 TO NPE_COUNT - 1 GENERATE
        npe_ce(i) <= agu_ce_out(i) AND ce;

        npe_0 : npe
        GENERIC MAP
        (
            NUM_REGS_16b => NUM_REGS_16b,
            NUM_REGS_4b => NUM_REGS_4b,
            DATA_WIDTH => DATA_WIDTH
        )
        PORT MAP
        (
            clk_i => clk_i,
            rstn_i => rstn_i,
            ce => npe_ce(i),
            sram_in => sram_data_in((i + 1) * DATA_WIDTH - 1 DOWNTO i * DATA_WIDTH),
            riscv_in => riscv_data_npe,
            rs1_addr => rs1_addr,
            rs2_addr => rs2_addr,
            rd_addr => rd_addr,
            arith_sel => arith_sel,
            rd_in_sel => rd_in_sel,
            ba_add_ena => ba_add_ena,
            data_out => npe_data_out((i + 1) * DATA_WIDTH - 1 DOWNTO i * DATA_WIDTH)
        );

        agu_0 : agu
        GENERIC MAP
        (
            DATA_WIDTH => DATA_WIDTH,
            BASE_ADDR_WIDTH => BASE_ADDR_WIDTH,
            BASE_ADDR_COUNT => BASE_ADDR_COUNT,
            NPE_COUNT => NPE_COUNT
        )
        PORT MAP
        (
            clk => clk_i,
            rstn_i => rstn_i,
            ba_ret => ba_ret_id,
            ba_ret_ena => ba_ret_ena_id,
            ba_incr => ba_incr_id,
            ba_incr_ena => ba_incr_ena_id,
            ba_add_idx => ba_add_idx,
            ba_add_val => npe_data_out((i + 1) * DATA_WIDTH - 1 DOWNTO i * DATA_WIDTH),
            ba_add_ena => ba_add_ena,
            ba_add_dis_pe => ba_add_dis_pe,
            ena_pe => pe_ena,
            rel_addr => rel_addr,
            ba_out_idx => ba_out_idx,
            ba_out_ena => ba_out_ena,
            addr_out => sram_addr((i + 1) * 24 - 1 DOWNTO i * 24),
            pe_ena_out => agu_ce_out(i)
        );
    END GENERATE;

    sram_pe_ena <= agu_ce_out;
    sram_data_out <= npe_data_out;

END ARCHITECTURE;