LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY adder IS
    GENERIC (
        DATA_WIDTH : INTEGER := 16
    );
    PORT (
        op1 : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        op2 : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        ena : IN STD_LOGIC;
        sum : OUT STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0)
    );
END ENTITY;
ARCHITECTURE comb OF adder IS

    SIGNAL add_in_left : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL add_in_right : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
BEGIN
    -- Zero gate operands
    add_in_left <= op1 WHEN ena = '1'ELSE
        (OTHERS => '0');
    add_in_right <= op2 WHEN ena = '1' ELSE
        (OTHERS => '0');
    sum <= STD_LOGIC_VECTOR(signed(add_in_left) + signed(add_in_right));
END ARCHITECTURE comb;