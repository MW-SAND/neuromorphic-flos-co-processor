library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_loopcache is
  generic (
    LD_WIDTH : natural := 5 -- lower displacement part of width branch.
                            -- Indicates how large the loop cache array is (2^LD_WIDTH)
                            -- and the size of the loops which can be cached (2^LD_WIDTH instructions)
  );
  port (
    clk_i           : in  std_ulogic;
    rstn_i          : in  std_ulogic;
    -- control signals from/to cpu control --
    decode_valid_i  : in  std_ulogic;
    pc_i            : in  std_ulogic_vector(XLEN - 1 downto 0);
    opcode_i        : in  std_ulogic_vector(6 downto 0);      -- instruction opcode
    imm_i           : in  std_ulogic_vector(XLEN-1 downto 0); -- instruction immediate
    branch_taken_i  : in  std_ulogic;
    fetch_disable_o : out std_ulogic;
    -- bus in  --
    valid_i         : in  std_ulogic_vector(1 downto 0);
    ack_i           : in  std_ulogic;
    data_i          : in  std_ulogic_vector((2 + 32) - 1 downto 0);
    -- bus out --
    valid_o         : out std_ulogic_vector(1 downto 0);
    ack_o           : out std_ulogic;
    data_o          : out std_ulogic_vector((2 + 32) - 1 downto 0)
  );
end neorv32_cpu_loopcache;

architecture neorv32_cpu_loopcache_rtl of neorv32_cpu_loopcache is

  type state_t is (IDLE, FILL, ACTIVE);
  signal state : state_t;

  signal count_register : std_ulogic_vector(LD_WIDTH-1 downto 0);

  signal op_is_cof : std_ulogic; -- instruction is change of flow (cof)
  signal op_is_sbb : std_ulogic; -- instruction is short backward branch (sbb)
  signal imm_ud    : std_ulogic_vector(XLEN-LD_WIDTH-3 downto 0); -- immediate upper displacement
  signal imm_ld    : std_ulogic_vector(LD_WIDTH-1 downto 0);      -- immediate lower displacement

  type loopcache_t is array (0 to (2**LD_WIDTH)-1) of std_ulogic_vector((2 + 32) - 1 downto 0);

  signal loopcache_mem : loopcache_t;
  signal loopcache_index : integer;

begin

  loopcache_fsm: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      state <= IDLE;
      count_register <= (others => '0');
    elsif rising_edge(clk_i) then
      if decode_valid_i = '1' then
        case state is

          when IDLE =>
            if (op_is_sbb = '1') and (branch_taken_i = '1') then
              count_register <= imm_ld;
              state <= FILL;
            end if;

          when FILL =>
            count_register <= std_ulogic_vector(signed(count_register) + 1);

            if (op_is_cof = '1') then
              if signed(count_register) = 0 then -- triggering sbb
                if (branch_taken_i = '1') then
                  count_register <= imm_ld;
                  state <= ACTIVE;
                else -- branch not taken
                  state <= IDLE;
                end if;
              else
                if (branch_taken_i = '1') then
                  state <= IDLE;
                end if;
              end if;
            end if;

          when ACTIVE =>
            count_register <= std_ulogic_vector(signed(count_register) + 1);

            if (op_is_cof = '1') then
              if signed(count_register) = 0 then -- triggering sbb
                if (branch_taken_i = '1') then
                  count_register <= imm_ld;
                else -- branch not taken
                  state <= IDLE;
                end if;
              else
                if (branch_taken_i = '1') then
                  state <= IDLE;
                end if;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process loopcache_fsm;

  -- connect immidiate upper and lower displacement byte aligned
  imm_ld <= imm_i(LD_WIDTH+1 downto 2);
  imm_ud <= imm_i(XLEN-1 downto LD_WIDTH+2);

  -- check for change of flow (cof) and short backward branch (sbb) instructions
  op_is_cof <= '1' when ((opcode_i = opcode_branch_c) or (opcode_i = opcode_jal_c) or (opcode_i = opcode_jalr_c))
               else '0';
  op_is_sbb <= '1' when ((opcode_i = opcode_branch_c) and (imm_ud = std_ulogic_vector(to_signed(-1, imm_ud'length))))
               else '0';

  loopcache_index <= to_integer(unsigned(pc_i(LD_WIDTH+1 downto 2)));

  loopcache: process(rstn_i, clk_i)
  begin
    if rising_edge(clk_i) then
      if (state = FILL) and (ack_i ='1') then
        loopcache_mem(loopcache_index) <= data_i;
      end if;
    end if;
  end process loopcache;

  -- Output signals, repace fetch engine with loopcache when in active state
  valid_o <= "11" when (state = ACTIVE) else valid_i;
  ack_o <= '0' when (state = ACTIVE) else ack_i;
  data_o <= loopcache_mem(loopcache_index) when (state = ACTIVE) else data_i;
  fetch_disable_o <= '1' when (state = ACTIVE) else '0';

end neorv32_cpu_loopcache_rtl;
