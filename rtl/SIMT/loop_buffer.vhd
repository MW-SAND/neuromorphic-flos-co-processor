LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

USE work.neorv32_package.ALL;

USE work.math_utils.ALL;

ENTITY loop_buffer IS
    GENERIC (
        NUM_REGS_16b : INTEGER := 8;
        NUM_REGS_4b : INTEGER := 8;
        DATA_WIDTH : INTEGER := 16;
        BASE_ADDR_WIDTH : INTEGER := 20;
        TASK_BUFFER_SIZE : INTEGER := 16;
        TASK_SIZE : INTEGER := 24;
        IRQ_EMPTY_OFFSET : INTEGER := 4;
        NPE_COUNT : INTEGER := 8
    );
    PORT (
        clk_i : IN STD_LOGIC;
        stall_rq : IN STD_LOGIC;
        rstn_i : IN STD_LOGIC;
        bus_req_i : IN bus_req_t;
        agu_ce_in : IN STD_LOGIC_VECTOR(NPE_COUNT - 1 DOWNTO 0);
        bus_rsp_o : OUT bus_rsp_t;
        decode_ena : OUT STD_LOGIC;
        irq_o : OUT STD_LOGIC;
        cur_instr_out : OUT STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
        ba_ret_out : OUT STD_LOGIC_VECTOR(2 * BASE_ADDR_WIDTH - 1 DOWNTO 0);
        ba_incr_out : OUT STD_LOGIC_VECTOR(2 * 16 - 1 DOWNTO 0);
        ba_ret_ena_out : OUT STD_LOGIC;
        ba_incr_ena_out : OUT STD_LOGIC;
        riscv_data : OUT STD_LOGIC_VECTOR(DATA_WIDTH - 1 DOWNTO 0);
        riscv_rd : OUT STD_LOGIC_VECTOR(log2(NUM_REGS_16b + NUM_REGS_4b) - 1 DOWNTO 0);
        riscv_ena : OUT STD_LOGIC
    );
END ENTITY;
ARCHITECTURE rtl OF loop_buffer IS

    TYPE rf_task_buffer_t IS ARRAY (0 TO TASK_BUFFER_SIZE - 1) OF STD_LOGIC_VECTOR(TASK_SIZE - 1 DOWNTO 0);
    SIGNAL rf_task_buffer : rf_task_buffer_t;
    SIGNAL rf_task_buffer_next : rf_task_buffer_t;

    SIGNAL cur_task : STD_LOGIC_VECTOR(TASK_SIZE - 1 DOWNTO 0);

    SIGNAL tb_read_idx : STD_LOGIC_VECTOR(log2(TASK_BUFFER_SIZE) - 1 DOWNTO 0);
    SIGNAL tb_read_idx_next : STD_LOGIC_VECTOR(log2(TASK_BUFFER_SIZE) - 1 DOWNTO 0);
    SIGNAL tb_read_idx_incr : STD_LOGIC_VECTOR(log2(TASK_BUFFER_SIZE) - 1 DOWNTO 0);

    SIGNAL tb_write_idx : STD_LOGIC_VECTOR(log2(TASK_BUFFER_SIZE) - 1 DOWNTO 0);
    SIGNAL tb_write_idx_next : STD_LOGIC_VECTOR(log2(TASK_BUFFER_SIZE) - 1 DOWNTO 0);
    SIGNAL tb_write_idx_incr : STD_LOGIC_VECTOR(log2(TASK_BUFFER_SIZE) - 1 DOWNTO 0);

    SIGNAL tb_read_write_diff_next : STD_LOGIC_VECTOR(log2(TASK_BUFFER_SIZE) DOWNTO 0);
    SIGNAL tb_read_write_diff : STD_LOGIC_VECTOR(log2(TASK_BUFFER_SIZE) DOWNTO 0);

    SIGNAL tb_full : STD_LOGIC;
    SIGNAL tb_full_next : STD_LOGIC;

    SIGNAL tb_read_eq_write : STD_LOGIC;

    SIGNAL task_finished : STD_LOGIC;

    TYPE rf_base_address_t IS ARRAY (0 TO 2 - 1) OF STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);
    SIGNAL rf_base_address_ret : rf_base_address_t;
    SIGNAL rf_base_address_ret_next : rf_base_address_t;

    TYPE rf_base_address_incr_t IS ARRAY (0 TO 16 - 1) OF STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
    SIGNAL rf_base_address_incr : rf_base_address_incr_t;
    SIGNAL rf_base_address_incr_next : rf_base_address_incr_t;

    TYPE rf_loop_cond_t IS ARRAY (0 TO 3 - 1) OF STD_LOGIC_VECTOR(20 - 1 DOWNTO 0);
    SIGNAL rf_loop_cond : rf_loop_cond_t;
    SIGNAL rf_loop_cond_next : rf_loop_cond_t;

    TYPE rf_loop_counters_t IS ARRAY (0 TO 3 - 1) OF STD_LOGIC_VECTOR(10 - 1 DOWNTO 0);
    SIGNAL rf_loop_counters : rf_loop_counters_t;
    SIGNAL rf_loop_counters_next : rf_loop_counters_t;

    TYPE rf_program_ctrl_t IS ARRAY (0 TO 2 - 1) OF STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
    SIGNAL rf_program_ctrl : rf_program_ctrl_t;
    SIGNAL rf_program_ctrl_next : rf_program_ctrl_t;

    TYPE rf_program_t IS ARRAY (0 TO 128 - 1) OF STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
    SIGNAL rf_program : rf_program_t;
    SIGNAL rf_program_next : rf_program_t;

    SIGNAL pc : STD_LOGIC_VECTOR(8 - 1 DOWNTO 0);
    SIGNAL pc_next : STD_LOGIC_VECTOR(8 - 1 DOWNTO 0);
    SIGNAL pc_incr : STD_LOGIC_VECTOR(8 - 1 DOWNTO 0);

    SIGNAL program_end_reached : STD_LOGIC;

    SIGNAL task_id : STD_LOGIC_VECTOR(0 DOWNTO 0);
    SIGNAL task_id_next : STD_LOGIC_VECTOR(0 DOWNTO 0);

    TYPE state_t IS (idle, init, running);
    SIGNAL state : state_t;
    SIGNAL state_next : state_t;

    SIGNAL base_addr_0_incr_in : STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
    SIGNAL base_addr_0_incr_out : STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);
    SIGNAL base_addr_1_incr_in : STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);
    SIGNAL base_addr_1_incr_out : STD_LOGIC_VECTOR(BASE_ADDR_WIDTH - 1 DOWNTO 0);

    SIGNAL counter_incr_in : STD_LOGIC_VECTOR(10 - 1 DOWNTO 0);
    SIGNAL counter_incr_out : STD_LOGIC_VECTOR(10 - 1 DOWNTO 0);

    SIGNAL cur_instr_is_branch : STD_LOGIC;

    SIGNAL loop_id : STD_LOGIC_VECTOR(2 - 1 DOWNTO 0);

    SIGNAL loop_cond_met : STD_LOGIC;

    SIGNAL cur_instr : STD_LOGIC_VECTOR(16 - 1 DOWNTO 0);

    SIGNAL base_addr_0_incr_index : STD_LOGIC_VECTOR(4 - 1 DOWNTO 0);
    SIGNAL base_addr_1_incr_index : STD_LOGIC_VECTOR(4 - 1 DOWNTO 0);

    CONSTANT ZERO_BIT : STD_LOGIC_VECTOR(0 DOWNTO 0) := "0";
    CONSTANT ONE_BIT : STD_LOGIC_VECTOR(0 DOWNTO 0) := "1";

    SIGNAL irq : STD_LOGIC;
    SIGNAL irq_next : STD_LOGIC;

    SIGNAL bus_req_cntr : STD_LOGIC_VECTOR(4 - 1 DOWNTO 0);
    SIGNAL bus_req_cntr_next : STD_LOGIC_VECTOR(4 - 1 DOWNTO 0);
    SIGNAL irq_empty_ena : STD_LOGIC;
    SIGNAL irq_empty_offset_ena : STD_LOGIC;
    SIGNAL irq_empty_ena_next : STD_LOGIC;
    SIGNAL irq_empty_offset_ena_next : STD_LOGIC;

    SIGNAL agu_ce : STD_LOGIC_VECTOR(NPE_COUNT - 1 DOWNTO 0);
    SIGNAL all_agu_disabled : STD_LOGIC;
    CONSTANT AGU_CE_COMP : STD_LOGIC_VECTOR(NPE_COUNT - 1 DOWNTO 0) := (OTHERS => '0');

BEGIN
    -- Check if all AGUs are disabled
    p_all_agu_disabled : PROCESS (agu_ce)
    BEGIN
        all_agu_disabled <= '0';

        IF (agu_ce = AGU_CE_COMP) THEN
            all_agu_disabled <= '1';
        END IF;
    END PROCESS;

    -- Determine the register values of the task buffer
    p_task_buffer : PROCESS (bus_req_i, rf_task_buffer, tb_read_eq_write, tb_full, tb_write_idx)
    BEGIN
        rf_task_buffer_next <= rf_task_buffer;

        -- handle bus request
        IF (bus_req_i.stb = '1' AND bus_req_i.addr(9 DOWNTO 1) = "000000000") THEN
            -- buffer not full check
            IF tb_read_eq_write = '0' OR tb_full = '0' THEN
                rf_task_buffer_next(to_integer(unsigned(tb_write_idx))) <= STD_LOGIC_VECTOR(bus_req_i.data(TASK_SIZE - 1 DOWNTO 0));
            END IF;
        END IF;
    END PROCESS;

    -- Select current task (if any)
    p_cur_task : PROCESS (rf_task_buffer, tb_read_eq_write, tb_full, tb_read_idx)
    BEGIN
        -- buffer not empty check
        IF tb_read_eq_write = '0' OR tb_full = '1' THEN
            cur_task <= rf_task_buffer(to_integer(unsigned(tb_read_idx)));
        ELSE
            cur_task <= (OTHERS => '0');
        END IF;
    END PROCESS;

    -- Determine read index of task buffer
    p_tb_read_idx : PROCESS (task_finished, tb_read_idx, tb_read_idx_incr)
    BEGIN
        IF task_finished = '1' THEN
            tb_read_idx_next <= tb_read_idx_incr;
        ELSE
            tb_read_idx_next <= tb_read_idx;
        END IF;
    END PROCESS;

    -- Determine write index of task buffer
    p_tb_write_idx : PROCESS (bus_req_i, tb_read_eq_write, tb_full, tb_write_idx, tb_write_idx_incr)
    BEGIN
        tb_write_idx_next <= tb_write_idx;

        -- handle bus request
        IF (bus_req_i.stb = '1' AND bus_req_i.addr(9 DOWNTO 1) = "000000000") THEN
            -- buffer not full check
            IF tb_read_eq_write = '0' OR tb_full = '0' THEN
                tb_write_idx_next <= tb_write_idx_incr;
            END IF;
        END IF;
    END PROCESS;

    -- Determine if the task buffer is full
    p_tb_full : PROCESS (tb_write_idx, tb_read_idx, tb_write_idx_next, tb_full, task_finished)
    BEGIN
        tb_full_next <= tb_full;

        -- Written to but not read from buffer, now write and read are equal
        IF (tb_write_idx /= tb_write_idx_next AND tb_write_idx_next = tb_read_idx AND task_finished = '0') THEN
            tb_full_next <= '1';

            -- Read from buffer
        ELSIF task_finished = '1' THEN
            tb_full_next <= '0';
        END IF;
    END PROCESS;

    -- Determine if task is finished
    p_task_finished : PROCESS (cur_task, program_end_reached, state)
    BEGIN
        IF (cur_task(1 DOWNTO 0) = "01" OR cur_task(1 DOWNTO 0) = "10" OR (state = running AND program_end_reached = '1')) THEN
            task_finished <= '1';
        ELSE
            task_finished <= '0';
        END IF;
    END PROCESS;

    irq_o <= irq;

    -- current instruction for instruction decoder
    cur_instr_out <= cur_instr;

    -- Enable the instruction decoder
    p_decode_ena : PROCESS (state, cur_instr)
    BEGIN
        -- Enable if running and not a branch instruction
        IF (state = running AND cur_instr(3 DOWNTO 0) /= "1000") THEN
            decode_ena <= '1';
        ELSE
            decode_ena <= '0';
        END IF;
    END PROCESS;

    -- Write value from RISCV to NPEs
    p_riscv : PROCESS (cur_task, state)
    BEGIN
        IF (state = idle AND cur_task(1 DOWNTO 0) = "10") THEN
            riscv_ena <= '1';
            riscv_data <= cur_task(TASK_SIZE - 1 DOWNTO TASK_SIZE - DATA_WIDTH);
            riscv_rd <= cur_task(TASK_SIZE - DATA_WIDTH - 1 DOWNTO TASK_SIZE - DATA_WIDTH - log2(NUM_REGS_16b + NUM_REGS_4b));
        ELSE
            riscv_ena <= '0';
            riscv_data <= (OTHERS => '0');
            riscv_rd <= (OTHERS => '0');
        END IF;
    END PROCESS;

    -- Determine next state
    p_state : PROCESS (state, cur_task, program_end_reached)
    BEGIN
        state_next <= state;
        task_id_next <= task_id;

        CASE (state) IS
            WHEN idle =>
                IF (cur_task(1 DOWNTO 0) = "11") THEN
                    state_next <= init;
                    task_id_next(0) <= cur_task(2);
                END IF;

            WHEN init =>
                state_next <= running;

            WHEN running =>
                IF (program_end_reached = '1') THEN
                    state_next <= idle;
                END IF;
        END CASE;
    END PROCESS;

    -- Write register file for the program
    p_rf_program : PROCESS (bus_req_i, rf_program, state)
    BEGIN
        rf_program_next <= rf_program;

        IF (bus_req_i.stb = '1' AND bus_req_i.rw = '1' AND state = idle) THEN
            -- 128 32-bit addresses for program instructions
            IF (bus_req_i.addr(9) = '1') THEN
                rf_program_next(to_integer(unsigned(bus_req_i.addr(7 DOWNTO 2)))) <= STD_LOGIC_VECTOR(bus_req_i.data(16 - 1 DOWNTO 0));
                rf_program_next(to_integer(unsigned(bus_req_i.addr(7 DOWNTO 2)) + 1)) <= STD_LOGIC_VECTOR(bus_req_i.data(32 - 1 DOWNTO 16));
            END IF;
        END IF;
    END PROCESS;

    -- Write register file for base address increments
    p_rf_base_address_incr : PROCESS (bus_req_i, rf_base_address_incr, state)
    BEGIN
        rf_base_address_incr_next <= rf_base_address_incr;

        IF (bus_req_i.stb = '1' AND bus_req_i.rw = '1' AND state = idle) THEN
            -- 64 32-bit addresses for base address increments
            IF bus_req_i.addr(9 DOWNTO 8) = "01" THEN
                rf_base_address_incr_next(to_integer(unsigned(bus_req_i.addr(7 DOWNTO 2)))) <= STD_LOGIC_VECTOR(bus_req_i.data(16 - 1 DOWNTO 0));
                rf_base_address_incr_next(to_integer(unsigned(bus_req_i.addr(7 DOWNTO 2)) + 1)) <= STD_LOGIC_VECTOR(bus_req_i.data(32 - 1 DOWNTO 16));
            END IF;
        END IF;
    END PROCESS;

    -- Write register file for loop conditions
    p_rf_loop_cond : PROCESS (bus_req_i, rf_loop_cond, state)
    BEGIN
        rf_loop_cond_next <= rf_loop_cond;

        IF (bus_req_i.stb = '1' AND bus_req_i.rw = '1' AND state = idle) THEN
            -- 8 32-bit addresses for loop conditions
            IF bus_req_i.addr(9 DOWNTO 5) = "00111" THEN
                rf_loop_cond_next(to_integer(unsigned(bus_req_i.addr(4 DOWNTO 2)))) <= STD_LOGIC_VECTOR(bus_req_i.data(20 - 1 DOWNTO 0));
            END IF;
        END IF;
    END PROCESS;

    -- Write register files for program control (task start, task end)
    p_rf_program_ctrl : PROCESS (bus_req_i, rf_program_ctrl, state)
    BEGIN
        rf_program_ctrl_next <= rf_program_ctrl;

        IF (bus_req_i.stb = '1' AND bus_req_i.rw = '1' AND state = idle) THEN
            -- 4 32-bit addresses for program ctrl 
            IF bus_req_i.addr(9 DOWNTO 4) = "001101" THEN
                rf_program_ctrl_next(to_integer(unsigned(bus_req_i.addr(3 DOWNTO 2)))) <= STD_LOGIC_VECTOR(bus_req_i.data(16 - 1 DOWNTO 0));
                rf_program_ctrl_next(to_integer(unsigned(bus_req_i.addr(3 DOWNTO 2)) + 1)) <= STD_LOGIC_VECTOR(bus_req_i.data(32 - 1 DOWNTO 16));
            END IF;
        END IF;
    END PROCESS;

    -- Write register file for retentive base addresses 
    p_rf_base_addr_ret : PROCESS (cur_task, rf_base_address_ret, state, cur_instr_is_branch)
    BEGIN
        rf_base_address_ret_next <= rf_base_address_ret;

        IF (cur_task(1 DOWNTO 0) = "01" AND state = idle) THEN
            rf_base_address_ret_next(to_integer(unsigned(cur_task(3 DOWNTO 2)))) <= cur_task(TASK_SIZE - 1 DOWNTO TASK_SIZE - BASE_ADDR_WIDTH);
        END IF;
    END PROCESS;

    -- Determine if base addresses of AGU should be overwritten
    p_ba_ret_ena : PROCESS (state)
    BEGIN
        IF (state = init) THEN
            ba_ret_ena_out <= '1';
        ELSE
            ba_ret_ena_out <= '0';
        END IF;
    END PROCESS;

    -- Send base addresses to AGU
    p_ba_ret_out : PROCESS (rf_base_address_ret)
    BEGIN
        FOR i IN 0 TO 2 - 1 LOOP
            ba_ret_out((i + 1) * BASE_ADDR_WIDTH - 1 DOWNTO i * BASE_ADDR_WIDTH) <= rf_base_address_ret(i);
        END LOOP;
    END PROCESS;

    -- Determine if base address should be incremented
    p_ba_incr_ena : PROCESS (state, cur_instr_is_branch)
    BEGIN
        IF (state = running AND cur_instr_is_branch = '1') THEN
            ba_incr_ena_out <= '1';
        ELSE
            ba_incr_ena_out <= '0';
        END IF;
    END PROCESS;

    -- Write register file for loop counters
    p_rf_loop_counters : PROCESS (state, counter_incr_out, cur_instr_is_branch, rf_loop_counters, loop_cond_met, cur_task, loop_id)
    BEGIN
        rf_loop_counters_next <= rf_loop_counters;

        IF (state = init) THEN
            rf_loop_counters_next <= (OTHERS => (OTHERS => '0'));

            FOR i IN 0 TO 3 - 1 LOOP
                rf_loop_counters_next(i)(4 - 1 DOWNTO 0) <= cur_task(4 * (i + 2) - 1 DOWNTO 4 * (i + 1));
            END LOOP;

        ELSIF (state = running AND cur_instr_is_branch = '1') THEN
            IF (loop_cond_met = '1') THEN
                rf_loop_counters_next(to_integer(unsigned(loop_id)))(10 - 1 DOWNTO 4) <= (OTHERS => '0');
                rf_loop_counters_next(to_integer(unsigned(loop_id)))(4 - 1 DOWNTO 0) <= cur_task(4 * (to_integer(unsigned(loop_id)) + 2) - 1 DOWNTO 4 * (to_integer(unsigned(loop_id)) + 1));
            ELSE
                rf_loop_counters_next(to_integer(unsigned(loop_id))) <= counter_incr_out;
            END IF;
        END IF;
    END PROCESS;

    -- Determine current instruction
    p_cur_instr : PROCESS (rf_program, pc, state)
    BEGIN
        IF (state = init) THEN
            cur_instr <= (OTHERS => '0');
        ELSE
            cur_instr <= rf_program(to_integer(unsigned(pc)));
        END IF;
    END PROCESS;

    -- Determine if current instruction is a branch instruction
    p_cur_instr_is_branch : PROCESS (cur_instr, state)
    BEGIN
        IF (state = running AND cur_instr(3 DOWNTO 0) = "1000") THEN
            cur_instr_is_branch <= '1';
        ELSE
            cur_instr_is_branch <= '0';
        END IF;
    END PROCESS;

    -- Determine the ID of the loop based on instruction
    p_loop_id : PROCESS (cur_instr_is_branch, cur_instr)
    BEGIN
        IF cur_instr_is_branch = '1' THEN
            loop_id <= cur_instr(5 DOWNTO 4);
        ELSE
            loop_id <= (OTHERS => '0');
        END IF;
    END PROCESS;

    -- Determine program counter
    p_pc : PROCESS (pc, pc_incr, state, rf_program_ctrl, cur_instr_is_branch, loop_cond_met, cur_instr, task_id, all_agu_disabled)
    BEGIN
        pc_next <= pc;

        IF (state = init) THEN
            pc_next <= rf_program_ctrl(to_integer(unsigned(task_id)))(16 - 1 DOWNTO 8);
        ELSIF (state = running) THEN
            IF (cur_instr_is_branch = '1' AND loop_cond_met = '0' AND all_agu_disabled = '0') THEN
                pc_next <= cur_instr(16 - 1 DOWNTO 8);
            ELSE
                pc_next <= pc_incr;
            END IF;
        END IF;
    END PROCESS;

    -- Determine IRQ request to RISCV
    p_irq : PROCESS (bus_req_i, irq, tb_read_idx, tb_read_idx_next, tb_write_idx, bus_req_cntr)
    BEGIN
        irq_next <= irq;

        -- read from buffer, next cycle buffer empty
        IF (tb_read_idx /= tb_read_idx_next AND bus_req_cntr = "1111") THEN
            IF (tb_read_idx_next = STD_LOGIC_VECTOR(unsigned(tb_write_idx) - IRQ_EMPTY_OFFSET)) THEN
                irq_next <= irq_empty_offset_ena;
            ELSIF (tb_read_idx_next = tb_write_idx) THEN
                irq_next <= irq_empty_ena;
            END IF;
        ELSIF (bus_req_i.rw = '1' AND bus_req_i.stb = '1') THEN
            irq_next <= '0';
        END IF;
    END PROCESS;

    -- Enable/Disable the IRQs 
    p_irq_empty_ena : PROCESS (bus_req_i, irq_empty_ena, irq_empty_offset_ena)
    BEGIN
        irq_empty_ena_next <= irq_empty_ena;
        irq_empty_offset_ena_next <= irq_empty_offset_ena;
        IF (bus_req_i.stb = '1' AND bus_req_i.rw = '1' AND bus_req_i.addr(9 DOWNTO 2) = "00000010") THEN
            irq_empty_ena_next <= bus_req_i.data(0);
            irq_empty_offset_ena_next <= bus_req_i.data(1);
        END IF;
    END PROCESS;

    -- bus request timer for IRQ
    p_bus_req_cntr : PROCESS (bus_req_i, bus_req_cntr)
    BEGIN
        IF bus_req_i.stb = '1' AND NOT(bus_req_i.addr(9 DOWNTO 2) = "00000010") THEN
            bus_req_cntr_next <= (OTHERS => '0');
        ELSIF bus_req_cntr = "1111" THEN
            bus_req_cntr_next <= bus_req_cntr;
        ELSE
            bus_req_cntr_next <= STD_LOGIC_VECTOR(unsigned(bus_req_cntr) + 1);
        END IF;
    END PROCESS;

    -- Adders and incrementers
    base_addr_0_incr_index <= task_id & loop_id & ZERO_BIT;
    base_addr_0_incr_in <= rf_base_address_incr(to_integer(unsigned(base_addr_0_incr_index)));
    ba_incr_out(16 - 1 DOWNTO 0) <= base_addr_0_incr_in;

    base_addr_1_incr_index <= task_id & loop_id & ONE_BIT;
    base_addr_1_incr_in <= rf_base_address_incr(to_integer(unsigned(base_addr_1_incr_index)));
    ba_incr_out(32 - 1 DOWNTO 16) <= base_addr_1_incr_in;

    counter_incr_in <= rf_loop_counters(to_integer(unsigned(loop_id)));
    counter_incr_out <= STD_LOGIC_VECTOR(unsigned(counter_incr_in) + 1);

    pc_incr <= STD_LOGIC_VECTOR(unsigned(pc) + 1);
    tb_read_idx_incr <= STD_LOGIC_VECTOR(unsigned(tb_read_idx) + 1);
    tb_write_idx_incr <= STD_LOGIC_VECTOR(unsigned(tb_write_idx) + 1);

    tb_read_write_diff_next <= tb_full & STD_LOGIC_VECTOR(unsigned(tb_write_idx) - unsigned(tb_read_idx));

    -- Comparators
    loop_cond_met <= '1' WHEN rf_loop_counters(to_integer(unsigned(loop_id))) >= rf_loop_cond(to_integer(unsigned(loop_id)))(10 * (1 + to_integer(unsigned(task_id))) - 1 DOWNTO 10 * to_integer(unsigned(task_id))) ELSE
        '0';
    program_end_reached <= '1' WHEN pc >= rf_program_ctrl(to_integer(unsigned(task_id)))(8 - 1 DOWNTO 0) ELSE
        '0';

    tb_read_eq_write <= '1' WHEN tb_read_idx = tb_write_idx ELSE
        '0';

    -- Update registers
    p_bus_handling : PROCESS (rstn_i, clk_i)
    BEGIN
        IF rstn_i = '0' THEN
            bus_rsp_o <= rsp_terminate_c;
            state <= idle;
            irq <= '0';
            rf_task_buffer <= (OTHERS => (OTHERS => '0'));
            tb_write_idx <= (OTHERS => '0');
            tb_read_idx <= (OTHERS => '0');
            tb_full <= '0';
            bus_req_cntr <= (OTHERS => '1');
            tb_read_write_diff <= (OTHERS => '0');
            irq_empty_ena <= '0';
            irq_empty_offset_ena <= '0';
            agu_ce <= (OTHERS => '1');
        ELSIF rising_edge(clk_i) THEN
            IF (state = idle OR bus_req_i.addr(9 DOWNTO 4) = "000000") THEN
                bus_rsp_o.ack <= bus_req_i.stb;
            ELSE
                bus_rsp_o.ack <= '0';
            END IF;

            IF (bus_req_i.stb = '1' AND bus_req_i.rw = '0') THEN
                tb_read_write_diff <= tb_read_write_diff_next;
                bus_rsp_o.data(log2(TASK_BUFFER_SIZE) DOWNTO 0) <= STD_ULOGIC_VECTOR(tb_read_write_diff_next);
            END IF;

            bus_req_cntr <= bus_req_cntr_next;

            bus_rsp_o.err <= '0';

            IF (bus_req_i.rw = '1' AND bus_req_i.stb = '1') THEN
                rf_task_buffer <= rf_task_buffer_next;
                tb_write_idx <= tb_write_idx_next;
                irq_empty_ena <= irq_empty_ena_next;
                irq_empty_offset_ena <= irq_empty_offset_ena_next;
            END IF;

            IF task_finished = '1' THEN
                tb_read_idx <= tb_read_idx_next;
            END IF;

            tb_full <= tb_full_next;

            IF (cur_task(1 DOWNTO 0) /= "00" AND state = idle) THEN
                rf_base_address_ret <= rf_base_address_ret_next;
                task_id <= task_id_next;
            END IF;

            IF (bus_req_i.rw = '1' AND state = idle) THEN
                rf_loop_cond <= rf_loop_cond_next;
                rf_program_ctrl <= rf_program_ctrl_next;
                rf_base_address_incr <= rf_base_address_incr_next;
                rf_program <= rf_program_next;
            END IF;

            IF (state /= idle AND stall_rq = '0') THEN
                pc <= pc_next;
                rf_loop_counters <= rf_loop_counters_next;
            END IF;

            state <= state_next;
            irq <= irq_next;
            agu_ce <= agu_ce_in;
        END IF;
    END PROCESS;
END ARCHITECTURE;