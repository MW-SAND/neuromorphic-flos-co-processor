LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.MATH_UTILS.ALL;

ENTITY register_file IS
    GENERIC (
        REG_DATA_WIDTH : INTEGER := 4;
        REG_NUM : INTEGER := 8;
        DATA_IN_DATA_WIDTH : INTEGER := 16;
        DATA_OUT_DATA_WIDTH : INTEGER := 16
    );
    PORT (
        wena : IN STD_LOGIC;
        reg_A_addr : IN STD_LOGIC_VECTOR(log2(REG_NUM) - 1 DOWNTO 0);
        reg_B_addr : IN STD_LOGIC_VECTOR(log2(REG_NUM) - 1 DOWNTO 0);
        waddr : IN STD_LOGIC_VECTOR(log2(REG_NUM) - 1 DOWNTO 0);
        data_in : IN STD_LOGIC_VECTOR(DATA_IN_DATA_WIDTH - 1 DOWNTO 0);
        clk_i : IN STD_LOGIC;
        ba_add_ena : IN STD_LOGIC;
        data_out_A : OUT STD_LOGIC_VECTOR(DATA_OUT_DATA_WIDTH - 1 DOWNTO 0);
        data_out_B : OUT STD_LOGIC_VECTOR(DATA_OUT_DATA_WIDTH - 1 DOWNTO 0)
    );
END register_file;
ARCHITECTURE rtl OF register_file IS
    TYPE registerFile IS ARRAY(0 TO REG_NUM - 1) OF STD_LOGIC_VECTOR(REG_DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL registers : registerFile;

    SIGNAL waddr_aligned : STD_LOGIC_VECTOR(log2(REG_NUM) - log2(DATA_IN_DATA_WIDTH/REG_DATA_WIDTH) - 1 DOWNTO 0);

    SIGNAL data_out_A_compressed, data_out_B_compressed : STD_LOGIC_VECTOR(REG_DATA_WIDTH - 1 DOWNTO 0);

    FUNCTION replicate(signal_bit : STD_LOGIC; count : NATURAL) RETURN STD_LOGIC_VECTOR IS
        VARIABLE result : STD_LOGIC_VECTOR(count - 1 DOWNTO 0);
    BEGIN
        FOR i IN 0 TO count - 1 LOOP
            result(i) := signal_bit;
        END LOOP;
        RETURN result;
    END FUNCTION;
BEGIN
    -- Select A and B ports
    data_out_A_compressed <= registers(to_integer(unsigned(reg_A_addr)));
    data_out_B_compressed <= registers(to_integer(unsigned(reg_B_addr)));

    -- Extend A and B
    extend_data_out : IF REG_DATA_WIDTH < DATA_OUT_DATA_WIDTH GENERATE
        data_out_A <= (replicate(data_out_A_compressed(REG_DATA_WIDTH - 1), DATA_OUT_DATA_WIDTH - REG_DATA_WIDTH) AND replicate(NOT(ba_add_ena), DATA_OUT_DATA_WIDTH - REG_DATA_WIDTH)) & data_out_A_compressed;
        data_out_B <= (replicate(data_out_B_compressed(REG_DATA_WIDTH - 1), DATA_OUT_DATA_WIDTH - REG_DATA_WIDTH) AND replicate(NOT(ba_add_ena), DATA_OUT_DATA_WIDTH - REG_DATA_WIDTH)) & data_out_B_compressed;
    END GENERATE extend_data_out;

    extend_data_out_n : IF REG_DATA_WIDTH >= DATA_OUT_DATA_WIDTH GENERATE
        data_out_A <= data_out_A_compressed;
        data_out_B <= data_out_B_compressed;
    END GENERATE extend_data_out_n;

    waddr_aligned <= waddr(log2(REG_NUM) - 1 DOWNTO log2(DATA_IN_DATA_WIDTH/REG_DATA_WIDTH));

    -- Write to register file upon write enable
    p_write : PROCESS (clk_i) IS
        VARIABLE addr_offset : STD_LOGIC_VECTOR(log2(DATA_IN_DATA_WIDTH/REG_DATA_WIDTH) - 1 DOWNTO 0);
        VARIABLE waddr_full : STD_LOGIC_VECTOR(log2(REG_NUM) - 1 DOWNTO 0);
    BEGIN
        IF rising_edge(clk_i) THEN
            -- clock enable
            IF wena = '1' THEN
                FOR i IN 0 TO DATA_IN_DATA_WIDTH/REG_DATA_WIDTH - 1 LOOP
                    addr_offset := STD_LOGIC_VECTOR(to_unsigned(i, log2(DATA_IN_DATA_WIDTH/REG_DATA_WIDTH)));
                    waddr_full := waddr_aligned & addr_offset;
                    registers(to_integer(unsigned(waddr_full))) <= data_in(REG_DATA_WIDTH * (i + 1) - 1 DOWNTO REG_DATA_WIDTH * i); -- Write
                END LOOP;
            END IF;
        END IF;
    END PROCESS;
END rtl;