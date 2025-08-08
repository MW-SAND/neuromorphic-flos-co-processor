LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.MATH_UTILS.ALL;

LIBRARY neorv32;
USE neorv32.neorv32_package.ALL;
USE neorv32.neorv32_data_image.ALL;

ENTITY neorv32_dmem_tc IS
  GENERIC (
    DMEM_SIZE : NATURAL; -- memory size in bytes, has to be a power of 2, min 4
    ACC_DATA_WIDTH : INTEGER := 16;
    PE_COUNT : INTEGER := 1
  );
  PORT (
    clk_i : IN STD_ULOGIC;
    rstn_i : IN STD_ULOGIC;
    bus_req_i : IN bus_req_t;
    bus_rsp_o : OUT bus_rsp_t;  
    acc_data_in : IN STD_LOGIC_VECTOR(PE_COUNT * ACC_DATA_WIDTH - 1 DOWNTO 0);
    acc_ena : IN STD_LOGIC;
    acc_rw : IN STD_LOGIC;
    acc_addr : IN STD_LOGIC_VECTOR(PE_COUNT * 24 - 1 DOWNTO 0);
    acc_pe_ena : IN STD_LOGIC_VECTOR(PE_COUNT - 1 DOWNTO 0);
    acc_data_out : OUT STD_LOGIC_VECTOR(PE_COUNT * ACC_DATA_WIDTH - 1 DOWNTO 0)
  );
END neorv32_dmem_tc;

ARCHITECTURE neorv32_dmem_rtl OF neorv32_dmem_tc IS
  SIGNAL bus_req_i_del : bus_req_t;

  COMPONENT sram IS
    GENERIC (
      sram_basename : STRING;
      banks : INTEGER := 4;
      log2_words_per_bank : INTEGER := 11;
      word_length : INTEGER := 16
    );
    PORT (
      addr : IN STD_LOGIC_VECTOR(log2(banks) + log2_words_per_bank - 1 DOWNTO 0);
      din : IN STD_LOGIC_VECTOR(word_length - 1 DOWNTO 0);
      we : IN STD_LOGIC_VECTOR(word_length/8 - 1 DOWNTO 0);
      ena : IN STD_LOGIC;
      clk : IN STD_LOGIC;
      dout : OUT STD_LOGIC_VECTOR(word_length - 1 DOWNTO 0)
    );
  END COMPONENT;

  FUNCTION rows_lower_bound(n : INTEGER) RETURN INTEGER IS
  BEGIN
    IF n < 2 THEN
      RETURN 2;
    ELSE
      RETURN n;
    END IF;
  END FUNCTION;

  CONSTANT num_rows : INTEGER := rows_lower_bound(PE_COUNT);

  CONSTANT sram_basename : STRING := "../rtl/core/hex_files/neorv32_data32_image";

  -- alternative memory description style --
  CONSTANT sram_c : BOOLEAN := true;

  -- local signals --
  SIGNAL riscv_addr, riscv_addr_incr : STD_LOGIC_VECTOR(index_size_f(DMEM_SIZE/4) DOWNTO 0);
  SIGNAL rdata : STD_ULOGIC_VECTOR(num_rows * ACC_DATA_WIDTH - 1 DOWNTO 0);
  SIGNAL rdata_logic : STD_LOGIC_VECTOR(num_rows * ACC_DATA_WIDTH - 1 DOWNTO 0);
  SIGNAL riscv_rden : STD_ULOGIC;
  SIGNAL riscv_stb_del : STD_LOGIC;
  SIGNAL acc_rden : STD_LOGIC;
  SIGNAL din : STD_LOGIC_VECTOR(num_rows * ACC_DATA_WIDTH - 1 DOWNTO 0);

  SIGNAL acc_row_ena : STD_LOGIC_VECTOR(num_rows - 1 DOWNTO 0);
  SIGNAL acc_row_ena_next : STD_LOGIC_VECTOR(num_rows - 1 DOWNTO 0);

  -- [NOTE] The memory (RAM) is built from 4 individual byte-wide memories as some synthesis tools
  --        have issues inferring 32-bit memories with individual byte-enable signals.
  -- [NOTE] Read-during-write behavior is irrelevant
  --        as read and write accesses are mutually exclusive (ensured by bus protocol).

  CONSTANT dmem_data_size_c : NATURAL := (data_init_image_b0'length) * 4; -- data (image) size in bytes

  CONSTANT words_per_bank : NATURAL := 2048;
  CONSTANT bytes_per_bank : NATURAL := 2 * words_per_bank;
  CONSTANT num_banks_needed : NATURAL := NATURAL(index_size_f(DMEM_SIZE / bytes_per_bank));
  CONSTANT num_banks : NATURAL := 2 ** num_banks_needed;
  CONSTANT bank_addr_width : NATURAL := log2(words_per_bank);

  SIGNAL sram_addr : STD_LOGIC_VECTOR(num_rows * (log2(num_banks) + bank_addr_width) - 1 DOWNTO 0);
  SIGNAL sram_we : STD_LOGIC_VECTOR(num_rows * 2 - 1 DOWNTO 0);
  SIGNAL sram_ena : STD_LOGIC;
  SIGNAL sram_row_ena : STD_LOGIC_VECTOR(num_rows - 1 DOWNTO 0);
  SIGNAL sram_row_ena_del : STD_LOGIC_VECTOR(num_rows - 1 DOWNTO 0);

  CONSTANT ZERO_BIT : STD_LOGIC_VECTOR(0 DOWNTO 0) := "0";
  CONSTANT ONE_BIT : STD_LOGIC_VECTOR(0 DOWNTO 0) := "1";

BEGIN
  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ASSERT NOT (dmem_data_size_c > DMEM_SIZE) REPORT
  "[NEORV32] Application image (" & NATURAL'image(dmem_data_size_c) &
  " bytes) does not fit into processor-internal DMEM (" &
  NATURAL'image(DMEM_SIZE) & " bytes)!" SEVERITY error;

  -- Memory Core ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  sram_default :
  IF sram_c GENERATE
    -- Address calculation
    riscv_addr_incr <= STD_LOGIC_VECTOR(bus_req_i_del.addr(index_size_f(DMEM_SIZE/4) + 1 DOWNTO 2)) & ONE_BIT;
    riscv_addr <= STD_LOGIC_VECTOR(bus_req_i_del.addr(index_size_f(DMEM_SIZE/4) + 1 DOWNTO 2)) & ZERO_BIT;

    sram_ena <= riscv_stb_del OR bus_req_i_del.stb;

    sram_row_gen : FOR i IN 0 TO num_rows - 1 GENERATE
      sram_row_block : BLOCK
        CONSTANT sram_name : STRING := sram_basename;
      BEGIN

        extend_sram_addr : IF log2(num_banks) + bank_addr_width > index_size_f(DMEM_SIZE/4) + 1 GENERATE
          sram_addr((i + 1) * (log2(num_banks) + bank_addr_width) - 1 DOWNTO i * (log2(num_banks) + bank_addr_width) + index_size_f(DMEM_SIZE/4) + 1) <= (OTHERS => '0');
        END GENERATE;

        -- Generate SRAM row
        sram_block : sram
        GENERIC MAP(
          sram_basename => sram_name,
          banks => num_banks / num_rows,
          log2_words_per_bank => bank_addr_width,
          word_length => ACC_DATA_WIDTH
        )
        PORT MAP(
          addr => sram_addr((i + 1) * (log2(num_banks) + bank_addr_width) - 1 DOWNTO i * (log2(num_banks) + bank_addr_width) + log2(num_rows)),
          din => din((i + 1) * ACC_DATA_WIDTH - 1 DOWNTO i * ACC_DATA_WIDTH),
          we => sram_we((i + 1) * 2 - 1 DOWNTO i * 2),
          ena => sram_row_ena(i),
          clk => clk_i,
          dout => rdata_logic((i + 1) * ACC_DATA_WIDTH - 1 DOWNTO i * ACC_DATA_WIDTH)
        );
      END BLOCK;
    END GENERATE;

    rdata <= STD_ULOGIC_VECTOR(rdata_logic);
  END GENERATE;

  -- Bus Response ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_feedback : PROCESS (rstn_i, clk_i)
  BEGIN
    IF (rstn_i = '0') THEN
      riscv_rden <= '0';
      bus_rsp_o.ack <= '0';
      acc_rden <= '0';
      riscv_stb_del <= '0';
      acc_row_ena <= (OTHERS => '0');
      bus_req_i_del.stb <= '0';
    ELSIF rising_edge(clk_i) THEN
      IF bus_req_i_del.stb = '1' OR bus_req_i.stb = '1' THEN
        bus_req_i_del <= bus_req_i;
      END IF;

      -- Delay read requests of the riscv upon pending accelerator request
      IF bus_req_i_del.stb = '1' AND acc_ena = '1' THEN
        riscv_stb_del <= '1';
      ELSIF acc_ena = '0' THEN
        riscv_stb_del <= '0';
      END IF;

      acc_row_ena <= acc_row_ena_next;

      sram_row_ena_del <= sram_row_ena;

      -- Remember whether a read request has been made in the previous cycle
      riscv_rden <= (bus_req_i_del.stb OR riscv_stb_del) AND (NOT bus_req_i_del.rw) AND (NOT acc_ena);
      acc_rden <= acc_ena AND (NOT acc_rw);
      bus_rsp_o.ack <= (bus_req_i_del.stb OR riscv_stb_del) AND (NOT acc_ena);
    END IF;
  END PROCESS bus_feedback;

  bus_rsp_o.err <= '0'; -- no access error possible

  -- Select data out for the accelerators 
  acc_data_out_mul_pe : IF PE_COUNT >= 2 GENERATE
    gen_acc_data_out_sel : FOR i IN 0 TO num_rows - 1 GENERATE
      acc_data_out((i + 1) * ACC_DATA_WIDTH - 1 DOWNTO i * ACC_DATA_WIDTH) <= STD_LOGIC_VECTOR(rdata((i + 1) * 16 - 1 DOWNTO i * 16)) WHEN acc_rden = '1' ELSE
      (OTHERS => '0');
    END GENERATE;
  END GENERATE;

  acc_data_out_sin_pe : IF PE_COUNT < 2 GENERATE
    acc_data_out(ACC_DATA_WIDTH - 1 DOWNTO 0) <= STD_LOGIC_VECTOR(rdata(ACC_DATA_WIDTH * 2 - 1 DOWNTO ACC_DATA_WIDTH)) WHEN acc_row_ena(1) = '1' ELSE
    STD_LOGIC_VECTOR(rdata(ACC_DATA_WIDTH - 1 DOWNTO 0));
  END GENERATE;

  -- Select data out for the riscv
  p_bus_rsp_data : PROCESS (sram_row_ena_del, rdata, riscv_rden)
    VARIABLE idx : INTEGER := 0;
  BEGIN

    idx := 0;
    FOR i IN 0 TO num_rows - 1 LOOP
      IF sram_row_ena_del(i) = '1' THEN
        idx := i;
        EXIT;
      END IF;
    END LOOP;

    IF riscv_rden = '1' THEN
      bus_rsp_o.data <= rdata((idx + 2) * ACC_DATA_WIDTH - 1 DOWNTO idx * ACC_DATA_WIDTH);
    ELSE
      bus_rsp_o.data <= (OTHERS => '0');
    END IF;
  END PROCESS;

  -- Write Enable Generation
  p_we_gen : PROCESS (riscv_addr, riscv_addr_incr, acc_ena, bus_req_i_del, acc_rw)
  BEGIN
    IF num_banks > 1 THEN
      FOR i IN 0 TO num_rows - 1 LOOP
        IF (i = to_integer(unsigned(riscv_addr(log2(num_rows) - 1 DOWNTO 0)))) AND acc_ena = '0' THEN
          sram_we(i * 2) <= bus_req_i_del.ben(0) AND bus_req_i_del.rw;
          sram_we(i * 2 + 1) <= bus_req_i_del.ben(1) AND bus_req_i_del.rw;
        ELSIF (i = to_integer(unsigned(riscv_addr_incr(log2(num_rows) - 1 DOWNTO 0)))) AND acc_ena = '0' THEN
          sram_we(i * 2) <= bus_req_i_del.ben(2) AND bus_req_i_del.rw;
          sram_we(i * 2 + 1) <= bus_req_i_del.ben(3) AND bus_req_i_del.rw;
        ELSE
          sram_we(i * 2) <= acc_rw AND acc_ena;
          sram_we(i * 2 + 1) <= acc_rw AND acc_ena;
        END IF;
      END LOOP;
    ELSE -- not implemented
      sram_we <= (OTHERS => '0');
    END IF;
  END PROCESS;

  -- Address Propagation
  p_addr_gen : PROCESS (riscv_addr, riscv_addr_incr, acc_ena, acc_addr)
  BEGIN
    IF num_banks > 1 THEN
      FOR i IN 0 TO num_rows - 1 LOOP
        IF (i = to_integer(unsigned(riscv_addr(log2(num_rows) - 1 DOWNTO 0)))) AND acc_ena = '0' THEN
          sram_addr(i * (log2(num_banks) + bank_addr_width) + index_size_f(DMEM_SIZE/4) DOWNTO i * (log2(num_banks) + bank_addr_width)) <= riscv_addr(index_size_f(DMEM_SIZE/4) DOWNTO 0);
        ELSIF (i = to_integer(unsigned(riscv_addr_incr(log2(num_rows) - 1 DOWNTO 0)))) AND acc_ena = '0' THEN
          sram_addr(i * (log2(num_banks) + bank_addr_width) + index_size_f(DMEM_SIZE/4) DOWNTO i * (log2(num_banks) + bank_addr_width)) <= riscv_addr_incr(index_size_f(DMEM_SIZE/4) DOWNTO 0);
        ELSE
          sram_addr(i * (log2(num_banks) + bank_addr_width) + index_size_f(DMEM_SIZE/4) DOWNTO i * (log2(num_banks) + bank_addr_width)) <= acc_addr(i * 24 + index_size_f(DMEM_SIZE/4) + 1 DOWNTO i * 24 + 1);
        END IF;
      END LOOP;
    ELSE -- not implemented
      sram_addr <= (OTHERS => '0');
    END IF;
  END PROCESS;

  -- Enable sram rows for invidiual processing elements
  p_acc_row_ena : PROCESS (acc_ena, acc_addr, acc_pe_ena)
  BEGIN
    IF (PE_COUNT >= 2) THEN
      FOR i IN 0 TO PE_COUNT - 1 LOOP
        acc_row_ena_next(i) <= acc_ena AND acc_pe_ena(i);
      END LOOP;
    ELSE
      acc_row_ena_next(0) <= acc_pe_ena(0) AND acc_ena AND NOT(acc_addr(1));
      acc_row_ena_next(1) <= acc_pe_ena(0) AND acc_ena AND acc_addr(1);
    END IF;
  END PROCESS;

  -- Enable SRAM rows for accelerator and riscv
  p_ena_gen : PROCESS (riscv_addr, riscv_addr_incr, sram_ena, acc_row_ena_next, acc_ena)
  BEGIN
    -- Fixes null argument in function error
    IF num_banks > 1 THEN
      FOR i IN 0 TO num_rows - 1 LOOP
        IF (i = to_integer(unsigned(riscv_addr(log2(num_rows) - 1 DOWNTO 0)))) THEN
          sram_row_ena(i) <= (sram_ena AND NOT(acc_ena)) OR acc_row_ena_next(i);
        ELSIF (i = to_integer(unsigned(riscv_addr_incr(log2(num_rows) - 1 DOWNTO 0)))) THEN
          sram_row_ena(i) <= (sram_ena AND NOT (acc_ena)) OR acc_row_ena_next(i);
        ELSE
          sram_row_ena(i) <= acc_row_ena_next(i);
        END IF;
      END LOOP;
    ELSE
      sram_row_ena(0) <= sram_ena OR acc_ena;
    END IF;
  END PROCESS;

  -- Data In Propagation
  p_din_gen : PROCESS (riscv_addr, riscv_addr_incr, acc_ena, acc_data_in, bus_req_i_del)
  BEGIN
    IF num_banks > 1 THEN
      FOR i IN 0 TO num_rows - 1 LOOP
        IF (i = to_integer(unsigned(riscv_addr(log2(num_rows) - 1 DOWNTO 0)))) AND acc_ena = '0' THEN
          din((i + 1) * ACC_DATA_WIDTH - 1 DOWNTO i * ACC_DATA_WIDTH) <= STD_LOGIC_VECTOR(bus_req_i_del.data(ACC_DATA_WIDTH - 1 DOWNTO 0));
        ELSIF (i = to_integer(unsigned(riscv_addr_incr(log2(num_rows) - 1 DOWNTO 0)))) AND acc_ena = '0' THEN
          din((i + 1) * ACC_DATA_WIDTH - 1 DOWNTO i * ACC_DATA_WIDTH) <= STD_LOGIC_VECTOR(bus_req_i_del.data(ACC_DATA_WIDTH * 2 - 1 DOWNTO ACC_DATA_WIDTH));
        ELSE
          IF PE_COUNT >= 2 THEN
            din((i + 1) * ACC_DATA_WIDTH - 1 DOWNTO i * ACC_DATA_WIDTH) <= acc_data_in((i + 1) * ACC_DATA_WIDTH - 1 DOWNTO i * ACC_DATA_WIDTH);
          ELSE
            din((i + 1) * ACC_DATA_WIDTH - 1 DOWNTO i * ACC_DATA_WIDTH) <= acc_data_in(ACC_DATA_WIDTH - 1 DOWNTO 0);
          END IF;
        END IF;
      END LOOP;
    ELSE -- not implemented
      din <= (OTHERS => '0');
    END IF;
  END PROCESS;

END neorv32_dmem_rtl;