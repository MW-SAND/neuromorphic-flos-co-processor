LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY multiplier IS
    GENERIC (
        DATA_WIDTH : INTEGER := 16
    );
    PORT (
        op1 : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        op2 : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        ena : IN STD_LOGIC;
        prod : OUT STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0)
    );
END ENTITY;
ARCHITECTURE comb OF multiplier IS

    SIGNAL mult_in_left : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL mult_in_right : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL mult_out : STD_LOGIC_VECTOR(DATA_WIDTH * 2 - 1 DOWNTO 0);
BEGIN

    -- Zero-gate the operands
    mult_in_left <= op1 WHEN ena = '1'ELSE
        (OTHERS => '0');
    mult_in_right <= op2 WHEN ena = '1' ELSE
        (OTHERS => '0');
    mult_out <= STD_LOGIC_VECTOR(signed(mult_in_left) * signed(mult_in_right));
    prod <= mult_out(DATA_WIDTH - 1 DOWNTO 0);
END ARCHITECTURE comb;