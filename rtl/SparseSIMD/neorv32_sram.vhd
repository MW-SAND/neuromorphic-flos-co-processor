LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.numeric_std.ALL;
USE WORK.MATH_UTILS.ALL;

ENTITY sram IS
    GENERIC (
        sram_basename : STRING;
        banks : INTEGER := 32;
        log2_words_per_bank : INTEGER := 13;
        word_length : INTEGER := 32
    );
    PORT (
        addr : IN STD_LOGIC_VECTOR(log2(banks) + log2_words_per_bank - 1 DOWNTO 0);
        din : IN STD_LOGIC_VECTOR(word_length - 1 DOWNTO 0);
        we : IN STD_LOGIC_VECTOR(word_length/8 - 1 DOWNTO 0);
        ena : IN STD_LOGIC;
        clk : IN STD_LOGIC;
        dout : OUT STD_LOGIC_VECTOR(word_length - 1 DOWNTO 0)
    );
END sram;
ARCHITECTURE rtl OF sram IS
    SIGNAL ena_bank : STD_LOGIC_VECTOR(banks - 1 DOWNTO 0);
    SIGNAL ena_bank_del : STD_LOGIC_VECTOR(banks - 1 DOWNTO 0);
    SIGNAL dout_bank : STD_LOGIC_VECTOR(banks * word_length - 1 DOWNTO 0);

BEGIN
    -- Generate memory banks
    gen_banks : FOR i IN banks - 1 DOWNTO 0 GENERATE
        p_ena_gen : PROCESS (addr, ena)
        BEGIN
            -- Fixes null argument in function error
            IF banks > 1 THEN
                IF (i = to_integer(unsigned(addr(log2(banks) + log2_words_per_bank - 1 DOWNTO log2_words_per_bank)))) THEN
                    ena_bank(i) <= ena;
                ELSE
                    ena_bank(i) <= '0';
                END IF;
            ELSE
                ena_bank(i) <= ena;
            END IF;
        END PROCESS;

        gen_16b_sram : IF word_length = 16 GENERATE
            -- Generate 16 x 2048 SRAM bank
        END GENERATE;

        gen_32b_sram : IF word_length = 32 GENERATE
            -- Generate 32 x 1024 SRAM banks
        END GENERATE;

    END GENERATE;

    -- Select output of banks based on enable signal
    PROCESS (ena_bank_del, dout_bank)
    BEGIN
        dout <= (OTHERS => '0');
        FOR i IN banks - 1 DOWNTO 0 LOOP
            IF ena_bank_del(i) = '1' THEN
                dout <= dout_bank((i + 1) * word_length - 1 DOWNTO i * word_length);
            END IF;
        END LOOP;
    END PROCESS;

    PROCESS (clk)
    BEGIN
        IF rising_edge(clk) THEN
            ena_bank_del <= ena_bank;
        END IF;
    END PROCESS;

END rtl;