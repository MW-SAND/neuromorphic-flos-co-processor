LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.math_utils.ALL;

ENTITY agu IS
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
END ENTITY;
ARCHITECTURE rtl OF agu IS
    SIGNAL ba_out : STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);

    CONSTANT SRAM_EXT : STD_LOGIC_VECTOR(24 - BASE_ADDR_WIDTH - 1 DOWNTO 0) := (OTHERS => '0');

    SIGNAL ba_var : STD_LOGIC_VECTOR(BASE_ADDR_WIDTH * BASE_ADDR_COUNT - 1 DOWNTO 0);
    SIGNAL ba_var_next : STD_LOGIC_VECTOR(BASE_ADDR_WIDTH * BASE_ADDR_COUNT - 1 DOWNTO 0);

    SIGNAL rel_addr_ext : STD_LOGIC_VECTOR(log2(NPE_COUNT) + 1 + 5 - 1 DOWNTO 0);

    SIGNAL pe_ena : STD_LOGIC;
    SIGNAL pe_ena_next : STD_LOGIC;

    SIGNAL comp_zero : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0) := (OTHERS => '0');

    SIGNAL rel_addr_reg : STD_LOGIC_VECTOR(5 - 1 DOWNTO 0);

    FUNCTION replicate(signal_bit : STD_LOGIC; count : NATURAL) RETURN STD_LOGIC_VECTOR IS
        VARIABLE result : STD_LOGIC_VECTOR(count - 1 DOWNTO 0);
    BEGIN
        FOR i IN 0 TO count - 1 LOOP
            result(i) := signal_bit;
        END LOOP;
        RETURN result;
    END FUNCTION;
BEGIN
    pe_ena_out <= pe_ena;

    -- Extend the relative address
    rel_addr_ext(log2(NPE_COUNT) + 1 + 5 - 1 DOWNTO 5) <= (OTHERS => '0');
    rel_addr_ext(5 - 1 DOWNTO 0) <= rel_addr_reg;

    -- Determine memory address for SRAM
    ba_out <= ba_var(BASE_ADDR_WIDTH * (to_integer(unsigned(ba_out_idx)) + 1) - 1 DOWNTO BASE_ADDR_WIDTH * to_integer(unsigned(ba_out_idx))) WHEN ba_out_ena = '1' ELSE
        (OTHERS => '0');
    addr_out <= SRAM_EXT & STD_LOGIC_VECTOR(unsigned(ba_out) + shift_left(unsigned(rel_addr_ext), log2(NPE_COUNT) + 1));

    -- Determine next value of the base address
    p_ba_var : PROCESS (ba_var, ba_ret_ena, ba_ret, ba_incr_ena, ba_incr, ba_add_ena, ba_add_idx, ba_add_val, pe_ena)
        VARIABLE ba_add_val_ext : STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);
        VARIABLE ba_incr_ext : STD_LOGIC_VECTOR(BASE_ADDR_COUNT * BASE_ADDR_WIDTH - 1 DOWNTO 0);
    BEGIN
        ba_var_next <= ba_var;

        -- Load new base addresses from loop controller
        IF ba_ret_ena = '1' THEN
            ba_var_next <= ba_ret;

        -- Increment the base address with base address increments
        ELSIF ba_incr_ena = '1' AND pe_ena = '1' THEN
            FOR i IN 0 TO BASE_ADDR_COUNT - 1 LOOP
                ba_incr_ext((i + 1) * BASE_ADDR_WIDTH - 1 DOWNTO i * BASE_ADDR_WIDTH + 16) := replicate(ba_incr((i + 1) * 16 - 1), BASE_ADDR_WIDTH - 16);
                ba_incr_ext(i * BASE_ADDR_WIDTH + 16 - 1 DOWNTO i * BASE_ADDR_WIDTH) := ba_incr((i + 1) * 16 - 1 DOWNTO i * 16);
                ba_var_next((i + 1) * BASE_ADDR_WIDTH - 1 DOWNTO i * BASE_ADDR_WIDTH) <= STD_LOGIC_VECTOR(unsigned(ba_var((i + 1) * BASE_ADDR_WIDTH - 1 DOWNTO i * BASE_ADDR_WIDTH)) + shift_left(unsigned(ba_incr_ext((i + 1) * BASE_ADDR_WIDTH - 1 DOWNTO i * BASE_ADDR_WIDTH)), log2(NPE_COUNT) + 1));
            END LOOP;

        -- Add register value to base address
        ELSIF ba_add_ena = '1' AND pe_ena = '1' THEN
            ba_add_val_ext(BASE_ADDR_WIDTH - 1 DOWNTO DATA_WIDTH) := (OTHERS => '0');
            ba_add_val_ext(DATA_WIDTH - 1 DOWNTO 0) := ba_add_val;
            ba_var_next((to_integer(unsigned(ba_add_idx)) + 1) * BASE_ADDR_WIDTH - 1 DOWNTO to_integer(unsigned(ba_add_idx)) * BASE_ADDR_WIDTH) <= STD_LOGIC_VECTOR(unsigned(ba_var((to_integer(unsigned(ba_add_idx)) + 1) * BASE_ADDR_WIDTH - 1 DOWNTO to_integer(unsigned(ba_add_idx)) * BASE_ADDR_WIDTH)) + shift_left(unsigned(ba_add_val_ext), log2(NPE_COUNT) + 1));
        END IF;
    END PROCESS;

    -- Enable / Disable PE and SRAM based on register value
    p_pe_ena : PROCESS (ba_add_ena, ena_pe, ba_add_dis_pe, ba_add_val, pe_ena)
    BEGIN
        pe_ena_next <= pe_ena;

        IF (ba_add_dis_pe = '1' AND ba_add_ena = '1' AND ba_add_val = comp_zero) THEN
            pe_ena_next <= '0';
        ELSIF ena_pe = '1' THEN
            pe_ena_next <= '1';
        END IF;
    END PROCESS;

    -- Update registers 
    p_rf : PROCESS (rstn_i, clk)
    BEGIN
        IF rstn_i = '0' THEN
            ba_var <= (OTHERS => '0');
            pe_ena <= '1';
            rel_addr_reg <= (OTHERS => '0');
        ELSIF rising_edge(clk) THEN
            ba_var <= ba_var_next;
            pe_ena <= pe_ena_next;
            rel_addr_reg <= (OTHERS => '0');
        END IF;
    END PROCESS;

END ARCHITECTURE;