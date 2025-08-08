LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY max IS
    GENERIC (
        DATA_WIDTH : INTEGER := 16
    );
    PORT (
        op1 : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        op2 : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        ena : IN STD_LOGIC;
        max_op : OUT STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0)
    );
END ENTITY;
ARCHITECTURE comb OF max IS

    SIGNAL max_in_left : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL max_in_right : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
BEGIN

    -- Zero gate the operands
    max_in_left <= op1 WHEN ena = '1' ELSE
        (OTHERS => '0');
    max_in_right <= op2 WHEN ena = '1' ELSE
        (OTHERS => '0');
    max_op <= max_in_left WHEN signed(max_in_left) > signed(max_in_right) ELSE
        max_in_right;
END ARCHITECTURE comb;