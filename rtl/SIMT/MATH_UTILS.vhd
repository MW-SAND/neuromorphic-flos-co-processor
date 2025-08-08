LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;

PACKAGE MATH_UTILS IS
  FUNCTION log2 (value : INTEGER) RETURN INTEGER;
  FUNCTION ceil_1 (quotient : INTEGER; dividend : INTEGER; divisor : INTEGER) RETURN INTEGER;
END PACKAGE;

PACKAGE BODY MATH_UTILS IS
  -- Log2 function
  FUNCTION log2 (value : INTEGER) RETURN INTEGER IS
  BEGIN
    FOR i IN 0 TO value LOOP
      IF (2 ** i >= value) THEN
        RETURN i;
      END IF;
    END LOOP;

    RETURN 0;
  END FUNCTION;

  -- Return 1 to round up quotients (ceil)
  FUNCTION ceil_1 (quotient : INTEGER; dividend : INTEGER; divisor : INTEGER) RETURN INTEGER IS
    VARIABLE reconstruction : INTEGER;
  BEGIN
    reconstruction := quotient * divisor;
    IF reconstruction < dividend THEN
      RETURN 1;
    ELSE
      RETURN 0;
    END IF;
  END FUNCTION;

END PACKAGE BODY;