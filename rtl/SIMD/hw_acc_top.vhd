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
        NPE_COUNT : INTEGER := 1
    );
    PORT (
        clk_i : IN STD_LOGIC;
        rstn_i : IN STD_LOGIC;
        bus_req_i : IN bus_req_t;
        bus_rsp_o : OUT bus_rsp_t;
        sram_data_in : IN STD_LOGIC_VECTOR(NPE_COUNT * DATA_WIDTH - 1 DOWNTO 0);
        sram_ena : OUT STD_LOGIC;
        sram_rw : OUT STD_LOGIC;
        sram_addr : OUT STD_LOGIC_VECTOR(24 - 1 DOWNTO 0);
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
            BASE_ADDR_WIDTH : INTEGER := 20
        );
        PORT (
            clk_i : IN STD_LOGIC;
            stall_rq : IN STD_LOGIC;
            rstn_i : IN STD_LOGIC;
            bus_req_i : IN bus_req_t;
            bus_rsp_o : OUT bus_rsp_t;
            decode_ena : OUT STD_LOGIC;
            irq_o : OUT STD_LOGIC;
            cur_instr_out : OUT STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
            base_addr_0_out : OUT STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);
            base_addr_1_out : OUT STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);
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
            base_addr_0 : IN STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);
            base_addr_1 : IN STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);
            riscv_data_in : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
            riscv_ena : IN STD_LOGIC;
            riscv_rd : IN STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
            riscv_data_out : OUT STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
            rs1_addr : OUT STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
            rs2_addr : OUT STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
            rd_addr : OUT STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
            arith_sel : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
            rd_in_sel : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
            ce : OUT STD_LOGIC;
            sram_addr : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
            sram_ena : OUT STD_LOGIC;
            sram_rw : OUT STD_LOGIC;
            stall_rq : OUT STD_LOGIC
        );
    END COMPONENT;

    SIGNAL stall_rq : STD_LOGIC;
    SIGNAL decode_ena : STD_LOGIC;
    SIGNAL cur_instr : STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
    SIGNAL base_addr_0 : STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);
    SIGNAL base_addr_1 : STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);
    SIGNAL riscv_data, riscv_data_npe : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL riscv_rd : STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
    SIGNAL riscv_ena : STD_LOGIC;

    SIGNAL rs1_addr, rs2_addr, rd_addr : STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
    SIGNAL arith_sel : STD_LOGIC_VECTOR(3 DOWNTO 0);
    SIGNAL rd_in_sel : STD_LOGIC_VECTOR(1 DOWNTO 0);
    SIGNAL ce : STD_LOGIC;

BEGIN

    -- Instantiate Loop Controller
    loop_buffer_0 : loop_buffer
    GENERIC MAP
    (
        NUM_REGS_16b => NUM_REGS_16b,
        NUM_REGS_4b => NUM_REGS_4b,
        DATA_WIDTH => DATA_WIDTH,
        BASE_ADDR_WIDTH => BASE_ADDR_WIDTH
    )
    PORT MAP
    (
        clk_i => clk_i,
        rstn_i => rstn_i,
        stall_rq => stall_rq,
        bus_req_i => bus_req_i,
        bus_rsp_o => bus_rsp_o,
        decode_ena => decode_ena,
        irq_o => irq_o,
        cur_instr_out => cur_instr,
        base_addr_0_out => base_addr_0,
        base_addr_1_out => base_addr_1,
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
        base_addr_0 => base_addr_0,
        base_addr_1 => base_addr_1,
        riscv_data_in => riscv_data,
        riscv_rd => riscv_rd,
        riscv_ena => riscv_ena,
        riscv_data_out => riscv_data_npe,
        sram_addr => sram_addr,
        sram_ena => sram_ena,
        sram_rw => sram_rw,
        stall_rq => stall_rq,
        rs1_addr => rs1_addr,
        rs2_addr => rs2_addr,
        rd_in_sel => rd_in_sel,
        rd_addr => rd_addr,
        arith_sel => arith_sel,
        ce => ce
    );

    -- Instantiate array of PE 
    gen_npe : FOR i IN 0 TO NPE_COUNT - 1 GENERATE
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
            ce => ce,
            sram_in => sram_data_in((i + 1) * DATA_WIDTH - 1 DOWNTO i * DATA_WIDTH),
            riscv_in => riscv_data_npe,
            rs1_addr => rs1_addr,
            rs2_addr => rs2_addr,
            rd_addr => rd_addr,
            arith_sel => arith_sel,
            rd_in_sel => rd_in_sel,
            data_out => sram_data_out((i + 1) * DATA_WIDTH - 1 DOWNTO i * DATA_WIDTH)
        );
    END GENERATE;

END ARCHITECTURE;