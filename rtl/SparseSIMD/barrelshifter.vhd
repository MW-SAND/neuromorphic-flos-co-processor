LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.math_utils.ALL;

ENTITY barrelshifter IS
    GENERIC (
        DATA_WIDTH : INTEGER := 16
    );
    PORT (
        op1 : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        op2 : IN STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        ena : IN STD_LOGIC;
        op1_shifted : OUT STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0)
    );
END ENTITY;
ARCHITECTURE comb OF barrelshifter IS

    SIGNAL shift_in_left : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL shift_in_right : STD_LOGIC_VECTOR(log2(DATA_WIDTH) DOWNTO 0);
    SIGNAL shift_in_right_inv : STD_LOGIC_VECTOR(log2(DATA_WIDTH) DOWNTO 0);
    SIGNAL shift_amt : STD_LOGIC_VECTOR(log2(DATA_WIDTH) DOWNTO 0);
    SIGNAL shift_dir : STD_LOGIC;
    SIGNAL shift_out : STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);

    FUNCTION replicate(signal_bit : STD_LOGIC; count : NATURAL) RETURN STD_LOGIC_VECTOR IS
        VARIABLE result : STD_LOGIC_VECTOR(count - 1 DOWNTO 0);
    BEGIN
        FOR i IN 0 TO count - 1 LOOP
            result(i) := signal_bit;
        END LOOP;
        RETURN result;
    END FUNCTION;
BEGIN

    -- Determine operands
    shift_in_left <= op1 WHEN ena = '1'ELSE
        (OTHERS => '0');
    shift_in_right <= op2(log2(DATA_WIDTH) DOWNTO 0) WHEN ena = '1' ELSE
        (OTHERS => '0');
    
    -- Negate second operand and determine direction
    shift_in_right_inv <= STD_LOGIC_VECTOR(signed(NOT(shift_in_right)) + 1);
    shift_dir <= op2(DATA_WIDTH - 1) WHEN ena = '1' ELSE
        '0';
    shift_amt <= shift_in_right WHEN shift_dir = '0' ELSE
        shift_in_right_inv;

    p_shift : PROCESS (shift_in_left, shift_in_right, shift_dir, shift_amt, shift_in_right_inv)
        VARIABLE shift_temp : STD_LOGIC_VECTOR(DATA_WIDTH * (log2(DATA_WIDTH) + 1) - 1 DOWNTO 0);
        VARIABLE index : INTEGER;
    BEGIN
        -- Initial shift, shift 0, 1 or -1 positions
        IF shift_amt(0) = '1' THEN
            IF shift_dir = '1' THEN
                shift_temp(DATA_WIDTH - 1 DOWNTO 0) := shift_in_left(DATA_WIDTH - 1) & shift_in_left(DATA_WIDTH - 1 DOWNTO 1);
            ELSE
                shift_temp(DATA_WIDTH - 1 DOWNTO 0) := shift_in_left(DATA_WIDTH - 2 DOWNTO 0) & '0';
            END IF;
        ELSE
            shift_temp(DATA_WIDTH - 1 DOWNTO 0) := shift_in_left;
        END IF;

        -- Subsequent shifts, shift 2**index positions based on second operand
        FOR i IN 1 TO log2(DATA_WIDTH) LOOP
            index := 2 ** i;

            IF shift_amt(i) = '1' THEN
                IF shift_dir = '1' THEN
                    shift_temp(DATA_WIDTH * (i + 1) - 1 DOWNTO DATA_WIDTH * i) := replicate(shift_in_left(DATA_WIDTH - 1), index) & shift_temp(DATA_WIDTH * i - 1 DOWNTO DATA_WIDTH * (i - 1) + index);
                ELSE
                    shift_temp(DATA_WIDTH * (i + 1) - 1 DOWNTO DATA_WIDTH * i) := shift_temp(DATA_WIDTH * i - 1 - index DOWNTO DATA_WIDTH * (i - 1)) & replicate('0', index);
                END IF;
            ELSE
                shift_temp(DATA_WIDTH * (i + 1) - 1 DOWNTO DATA_WIDTH * i) := shift_temp(DATA_WIDTH * i - 1 DOWNTO DATA_WIDTH * (i - 1));
            END IF;
        END LOOP;

        op1_shifted <= shift_temp(DATA_WIDTH * (log2(DATA_WIDTH) + 1) - 1 DOWNTO DATA_WIDTH * log2(DATA_WIDTH));
    END PROCESS;
END ARCHITECTURE comb;