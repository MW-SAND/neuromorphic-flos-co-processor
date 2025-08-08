-- ================================================================================ --
-- NEORV32 SoC - Processor Top Entity                                               --
-- -------------------------------------------------------------------------------- --
-- HQ:         https://github.com/stnolting/neorv32                                 --
-- Data Sheet: https://stnolting.github.io/neorv32                                  --
-- User Guide: https://stnolting.github.io/neorv32/ug                               --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY neorv32;
USE neorv32.neorv32_package.ALL;

ENTITY neorv32_top_acc IS
  GENERIC (
    -- Processor Clocking --
    CLOCK_FREQUENCY : NATURAL := 0; -- clock frequency of clk_i in Hz
    HART_BASE : NATURAL := 0; -- offset in HART_IDs

    -- Dual-Core Configuration --
    DUAL_CORE_EN : BOOLEAN := false; -- enable dual-core homogeneous SMP

    -- Boot Configuration --
    BOOT_MODE_SELECT : NATURAL RANGE 0 TO 2 := 0; -- boot configuration select (default = 0 = bootloader)
    BOOT_ADDR_CUSTOM : STD_ULOGIC_VECTOR(31 DOWNTO 0) := x"00000000"; -- custom CPU boot address (if boot_config = 1)

    -- On-Chip Debugger (OCD) --
    OCD_EN : BOOLEAN := false; -- implement on-chip debugger
    OCD_AUTHENTICATION : BOOLEAN := false; -- implement on-chip debugger authentication
    OCD_JEDEC_ID : STD_ULOGIC_VECTOR(10 DOWNTO 0) := "00000000000"; -- JEDEC ID: continuation codes + vendor ID

    -- RISC-V CPU Extensions --
    RISCV_ISA_C : BOOLEAN := false; -- implement compressed extension
    RISCV_ISA_E : BOOLEAN := false; -- implement embedded RF extension
    RISCV_ISA_M : BOOLEAN := false; -- implement mul/div extension
    RISCV_ISA_U : BOOLEAN := false; -- implement user mode extension
    RISCV_ISA_Zaamo : BOOLEAN := false; -- implement atomic read-modify-write operations extension
    RISCV_ISA_Zalrsc : BOOLEAN := false; -- implement atomic reservation-set operations extension
    RISCV_ISA_Zba : BOOLEAN := false; -- implement shifted-add bit-manipulation extension
    RISCV_ISA_Zbb : BOOLEAN := false; -- implement basic bit-manipulation extension
    RISCV_ISA_Zbkb : BOOLEAN := false; -- implement bit-manipulation instructions for cryptography
    RISCV_ISA_Zbkc : BOOLEAN := false; -- implement carry-less multiplication instructions
    RISCV_ISA_Zbkx : BOOLEAN := false; -- implement cryptography crossbar permutation extension
    RISCV_ISA_Zbs : BOOLEAN := false; -- implement single-bit bit-manipulation extension
    RISCV_ISA_Zfinx : BOOLEAN := false; -- implement 32-bit floating-point extension
    RISCV_ISA_Zicntr : BOOLEAN := false; -- implement base counters
    RISCV_ISA_Zicond : BOOLEAN := false; -- implement integer conditional operations
    RISCV_ISA_Zihpm : BOOLEAN := false; -- implement hardware performance monitors
    RISCV_ISA_Zknd : BOOLEAN := false; -- implement cryptography NIST AES decryption extension
    RISCV_ISA_Zkne : BOOLEAN := false; -- implement cryptography NIST AES encryption extension
    RISCV_ISA_Zknh : BOOLEAN := false; -- implement cryptography NIST hash extension
    RISCV_ISA_Zksed : BOOLEAN := false; -- implement ShangMi block cipher extension
    RISCV_ISA_Zksh : BOOLEAN := false; -- implement ShangMi hash extension
    RISCV_ISA_Zmmul : BOOLEAN := false; -- implement multiply-only M sub-extension
    RISCV_ISA_Zxcfu : BOOLEAN := false; -- implement custom (instr.) functions unit

    -- Tuning Options --
    CPU_FAST_MUL_EN : BOOLEAN := false; -- use DSPs for M extension's multiplier
    CPU_FAST_SHIFT_EN : BOOLEAN := false; -- use barrel shifter for shift operations
    CPU_RF_HW_RST_EN : BOOLEAN := false; -- implement full hardware reset for register file

    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS : NATURAL RANGE 0 TO 16 := 0; -- number of regions (0..16)
    PMP_MIN_GRANULARITY : NATURAL := 4; -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    PMP_TOR_MODE_EN : BOOLEAN := false; -- implement TOR mode
    PMP_NAP_MODE_EN : BOOLEAN := false; -- implement NAPOT/NA4 modes

    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS : NATURAL RANGE 0 TO 13 := 0; -- number of implemented HPM counters (0..13)
    HPM_CNT_WIDTH : NATURAL RANGE 0 TO 64 := 40; -- total size of HPM counters (0..64)

    -- Internal Instruction memory (IMEM) --
    MEM_INT_IMEM_EN : BOOLEAN := false; -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE : NATURAL := 16 * 1024; -- size of processor-internal instruction memory in bytes (use a power of 2)

    -- Internal Data memory (DMEM) --
    MEM_INT_DMEM_EN : BOOLEAN := false; -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE : NATURAL := 8 * 1024; -- size of processor-internal data memory in bytes (use a power of 2)

    -- Internal Instruction Cache (iCACHE) --
    ICACHE_EN : BOOLEAN := false; -- implement instruction cache
    ICACHE_NUM_BLOCKS : NATURAL RANGE 1 TO 256 := 4; -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE : NATURAL RANGE 4 TO 2 ** 16 := 64; -- i-cache: block size in bytes (min 4), has to be a power of 2

    -- Internal Data Cache (dCACHE) --
    DCACHE_EN : BOOLEAN := false; -- implement data cache
    DCACHE_NUM_BLOCKS : NATURAL RANGE 1 TO 256 := 4; -- d-cache: number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE : NATURAL RANGE 4 TO 2 ** 16 := 64; -- d-cache: block size in bytes (min 4), has to be a power of 2

    -- External bus interface (XBUS) --
    XBUS_EN : BOOLEAN := false; -- implement external memory bus interface?
    XBUS_TIMEOUT : NATURAL := 255; -- cycles after a pending bus access auto-terminates (0 = disabled)
    XBUS_REGSTAGE_EN : BOOLEAN := false; -- add XBUS register stage
    XBUS_CACHE_EN : BOOLEAN := false; -- enable external bus cache (x-cache)
    XBUS_CACHE_NUM_BLOCKS : NATURAL RANGE 1 TO 256 := 64; -- x-cache: number of blocks (min 1), has to be a power of 2
    XBUS_CACHE_BLOCK_SIZE : NATURAL RANGE 1 TO 2 ** 16 := 32; -- x-cache: block size in bytes (min 4), has to be a power of 2

    -- Processor peripherals --
    IO_DISABLE_SYSINFO : BOOLEAN := false; -- disable the SYSINFO module (for advanced users only)
    IO_GPIO_NUM : NATURAL RANGE 0 TO 32 := 0; -- number of GPIO input/output pairs (0..32)
    IO_CLINT_EN : BOOLEAN := false; -- implement core local interruptor (CLINT)?
    IO_UART0_EN : BOOLEAN := false; -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART0_RX_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- RX FIFO depth, has to be a power of two, min 1
    IO_UART0_TX_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- TX FIFO depth, has to be a power of two, min 1
    IO_UART1_EN : BOOLEAN := false; -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_UART1_RX_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- RX FIFO depth, has to be a power of two, min 1
    IO_UART1_TX_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- TX FIFO depth, has to be a power of two, min 1
    IO_SPI_EN : BOOLEAN := false; -- implement serial peripheral interface (SPI)?
    IO_SPI_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- RTX FIFO depth, has to be a power of two, min 1
    IO_SDI_EN : BOOLEAN := false; -- implement serial data interface (SDI)?
    IO_SDI_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- RTX FIFO depth, has to be zero or a power of two, min 1
    IO_TWI_EN : BOOLEAN := false; -- implement two-wire interface (TWI)?
    IO_TWI_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- RTX FIFO depth, has to be zero or a power of two, min 1
    IO_TWD_EN : BOOLEAN := false; -- implement two-wire device (TWD)?
    IO_TWD_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- RTX FIFO depth, has to be zero or a power of two, min 1
    IO_PWM_NUM_CH : NATURAL RANGE 0 TO 16 := 0; -- number of PWM channels to implement (0..16)
    IO_WDT_EN : BOOLEAN := false; -- implement watch dog timer (WDT)?
    IO_TRNG_EN : BOOLEAN := false; -- implement true random number generator (TRNG)?
    IO_TRNG_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- data FIFO depth, has to be a power of two, min 1
    IO_CFS_EN : BOOLEAN := false; -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG : STD_ULOGIC_VECTOR(31 DOWNTO 0) := x"00000000"; -- custom CFS configuration generic
    IO_CFS_IN_SIZE : NATURAL := 32; -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE : NATURAL := 32; -- size of CFS output conduit in bits
    IO_NEOLED_EN : BOOLEAN := false; -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_NEOLED_TX_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- NEOLED FIFO depth, has to be a power of two, min 1
    IO_GPTMR_EN : BOOLEAN := false; -- implement general purpose timer (GPTMR)?
    IO_ONEWIRE_EN : BOOLEAN := false; -- implement 1-wire interface (ONEWIRE)?
    IO_ONEWIRE_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- RTX FIFO depth, has to be zero or a power of two, min 1
    IO_DMA_EN : BOOLEAN := false; -- implement direct memory access controller (DMA)?
    IO_SLINK_EN : BOOLEAN := false; -- implement stream link interface (SLINK)?
    IO_SLINK_RX_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- RX FIFO depth, has to be a power of two, min 1
    IO_SLINK_TX_FIFO : NATURAL RANGE 1 TO 2 ** 15 := 1; -- TX FIFO depth, has to be a power of two, min 1
    IO_CRC_EN : BOOLEAN := false -- implement cyclic redundancy check unit (CRC)?
  );
  PORT (
    -- Global control --
    clk_i : IN STD_ULOGIC; -- global clock, rising edge
    rstn_i : IN STD_ULOGIC; -- global reset, low-active, async
    rstn_ocd_o : OUT STD_ULOGIC; -- on-chip debugger reset output, low-active, sync
    rstn_wdt_o : OUT STD_ULOGIC; -- watchdog reset output, low-active, sync

    -- JTAG on-chip debugger interface (available if OCD_EN = true) --
    jtag_tck_i : IN STD_ULOGIC := 'L'; -- serial clock
    jtag_tdi_i : IN STD_ULOGIC := 'L'; -- serial data input
    jtag_tdo_o : OUT STD_ULOGIC; -- serial data output
    jtag_tms_i : IN STD_ULOGIC := 'L'; -- mode select

    -- External bus interface (available if XBUS_EN = true) --
    xbus_adr_o : OUT STD_ULOGIC_VECTOR(31 DOWNTO 0); -- address
    xbus_dat_o : OUT STD_ULOGIC_VECTOR(31 DOWNTO 0); -- write data
    xbus_tag_o : OUT STD_ULOGIC_VECTOR(2 DOWNTO 0); -- access tag
    xbus_we_o : OUT STD_ULOGIC; -- read/write
    xbus_sel_o : OUT STD_ULOGIC_VECTOR(3 DOWNTO 0); -- byte enable
    xbus_stb_o : OUT STD_ULOGIC; -- strobe
    xbus_cyc_o : OUT STD_ULOGIC; -- valid cycle
    xbus_dat_i : IN STD_ULOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => 'L'); -- read data
    xbus_ack_i : IN STD_ULOGIC := 'L'; -- transfer acknowledge
    xbus_err_i : IN STD_ULOGIC := 'L'; -- transfer error

    -- Stream Link Interface (available if IO_SLINK_EN = true) --
    slink_rx_dat_i : IN STD_ULOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => 'L'); -- RX input data
    slink_rx_src_i : IN STD_ULOGIC_VECTOR(3 DOWNTO 0) := (OTHERS => 'L'); -- RX source routing information
    slink_rx_val_i : IN STD_ULOGIC := 'L'; -- RX valid input
    slink_rx_lst_i : IN STD_ULOGIC := 'L'; -- RX last element of stream
    slink_rx_rdy_o : OUT STD_ULOGIC; -- RX ready to receive
    slink_tx_dat_o : OUT STD_ULOGIC_VECTOR(31 DOWNTO 0); -- TX output data
    slink_tx_dst_o : OUT STD_ULOGIC_VECTOR(3 DOWNTO 0); -- TX destination routing information
    slink_tx_val_o : OUT STD_ULOGIC; -- TX valid output
    slink_tx_lst_o : OUT STD_ULOGIC; -- TX last element of stream
    slink_tx_rdy_i : IN STD_ULOGIC := 'L'; -- TX ready to send

    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o : OUT STD_ULOGIC_VECTOR(31 DOWNTO 0); -- parallel output
    gpio_i : IN STD_ULOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => 'L'); -- parallel input; interrupt-capable

    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o : OUT STD_ULOGIC; -- UART0 send data
    uart0_rxd_i : IN STD_ULOGIC := 'L'; -- UART0 receive data
    uart0_rtsn_o : OUT STD_ULOGIC; -- HW flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_ctsn_i : IN STD_ULOGIC := 'L'; -- HW flow control: UART0.TX allowed to transmit, low-active, optional

    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o : OUT STD_ULOGIC; -- UART1 send data
    uart1_rxd_i : IN STD_ULOGIC := 'L'; -- UART1 receive data
    uart1_rtsn_o : OUT STD_ULOGIC; -- HW flow control: UART1.RX ready to receive ("RTR"), low-active, optional
    uart1_ctsn_i : IN STD_ULOGIC := 'L'; -- HW flow control: UART1.TX allowed to transmit, low-active, optional

    -- SPI (available if IO_SPI_EN = true) --
    spi_clk_o : OUT STD_ULOGIC; -- SPI serial clock
    spi_dat_o : OUT STD_ULOGIC; -- controller data out, peripheral data in
    spi_dat_i : IN STD_ULOGIC := 'L'; -- controller data in, peripheral data out
    spi_csn_o : OUT STD_ULOGIC_VECTOR(7 DOWNTO 0); -- chip-select, low-active

    -- SDI (available if IO_SDI_EN = true) --
    sdi_clk_i : IN STD_ULOGIC := 'L'; -- SDI serial clock
    sdi_dat_o : OUT STD_ULOGIC; -- controller data out, peripheral data in
    sdi_dat_i : IN STD_ULOGIC := 'L'; -- controller data in, peripheral data out
    sdi_csn_i : IN STD_ULOGIC := 'H'; -- chip-select, low-active

    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i : IN STD_ULOGIC := 'H'; -- serial data line sense input
    twi_sda_o : OUT STD_ULOGIC; -- serial data line output (pull low only)
    twi_scl_i : IN STD_ULOGIC := 'H'; -- serial clock line sense input
    twi_scl_o : OUT STD_ULOGIC; -- serial clock line output (pull low only)

    -- TWD (available if IO_TWD_EN = true) --
    twd_sda_i : IN STD_ULOGIC := 'H'; -- serial data line sense input
    twd_sda_o : OUT STD_ULOGIC; -- serial data line output (pull low only)
    twd_scl_i : IN STD_ULOGIC := 'H'; -- serial clock line sense input
    twd_scl_o : OUT STD_ULOGIC; -- serial clock line output (pull low only)

    -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
    onewire_i : IN STD_ULOGIC := 'H'; -- 1-wire bus sense input
    onewire_o : OUT STD_ULOGIC; -- 1-wire bus output (pull low only)

    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o : OUT STD_ULOGIC_VECTOR(15 DOWNTO 0); -- pwm channels

    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i : IN STD_ULOGIC_VECTOR(IO_CFS_IN_SIZE - 1 DOWNTO 0) := (OTHERS => 'L'); -- custom CFS inputs conduit
    cfs_out_o : OUT STD_ULOGIC_VECTOR(IO_CFS_OUT_SIZE - 1 DOWNTO 0); -- custom CFS outputs conduit

    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o : OUT STD_ULOGIC; -- async serial data line

    -- Machine timer system time (available if IO_CLINT_EN = true) --
    mtime_time_o : OUT STD_ULOGIC_VECTOR(63 DOWNTO 0); -- current system time

    -- CPU interrupts (for chip-internal usage only) --
    mtime_irq_i : IN STD_ULOGIC := 'L'; -- machine timer interrupt, available if IO_CLINT_EN = false
    msw_irq_i : IN STD_ULOGIC := 'L'; -- machine software interrupt, available if IO_CLINT_EN = false
    mext_irq_i : IN STD_ULOGIC := 'L' -- machine external interrupt
  );
END neorv32_top_acc;

ARCHITECTURE neorv32_top_rtl OF neorv32_top_acc IS

  -- ----------------------------------------------------------
  -- Boot Configuration (BOOT_MODE_SELECT)
  -- ----------------------------------------------------------
  -- 0: Internal bootloader ROM
  -- 1: Custom (use BOOT_ADDR_CUSTOM)
  -- 2: Internal IMEM initialized with application image
  -- ----------------------------------------------------------
  CONSTANT bootrom_en_c : BOOLEAN := BOOLEAN(BOOT_MODE_SELECT = 0);
  CONSTANT imem_as_rom_c : BOOLEAN := BOOLEAN(BOOT_MODE_SELECT = 2);
  CONSTANT cpu_boot_addr_c : STD_ULOGIC_VECTOR(31 DOWNTO 0) :=
  cond_sel_suv_f(BOOLEAN(BOOT_MODE_SELECT = 0), base_io_bootrom_c,
  cond_sel_suv_f(BOOLEAN(BOOT_MODE_SELECT = 1), BOOT_ADDR_CUSTOM,
  cond_sel_suv_f(BOOLEAN(BOOT_MODE_SELECT = 2), mem_imem_base_c, x"00000000")));
  -- auto-configuration --
  CONSTANT num_cores_c : NATURAL := cond_sel_natural_f(DUAL_CORE_EN, 2, 1);
  CONSTANT io_gpio_en_c : BOOLEAN := BOOLEAN(IO_GPIO_NUM > 0);
  CONSTANT io_pwm_en_c : BOOLEAN := BOOLEAN(IO_PWM_NUM_CH > 0);
  CONSTANT cpu_smpmp_c : BOOLEAN := BOOLEAN(PMP_NUM_REGIONS > 0);
  CONSTANT io_sysinfo_en_c : BOOLEAN := NOT IO_DISABLE_SYSINFO;

  -- make sure physical memory sizes are a power of two --
  CONSTANT imem_size_c : NATURAL := cond_sel_natural_f(is_power_of_two_f(MEM_INT_IMEM_SIZE), MEM_INT_IMEM_SIZE, 2 ** index_size_f(MEM_INT_IMEM_SIZE));
  CONSTANT dmem_size_c : NATURAL := cond_sel_natural_f(is_power_of_two_f(MEM_INT_DMEM_SIZE), MEM_INT_DMEM_SIZE, 2 ** index_size_f(MEM_INT_DMEM_SIZE));

  CONSTANT PE_COUNT : NATURAL := to_integer(unsigned(IO_CFS_CONFIG(7 DOWNTO 0)));

  -- reset nets --
  SIGNAL rstn_wdt, rstn_sys, rstn_ext : STD_ULOGIC;

  -- clock system --
  SIGNAL clk_gen : STD_ULOGIC_VECTOR(7 DOWNTO 0); -- scaled clock-enables
  --
  TYPE clk_gen_en_enum_t IS (
    CG_CFS, CG_UART0, CG_UART1, CG_SPI, CG_TWI, CG_TWD, CG_PWM, CG_WDT, CG_NEOLED, CG_GPTMR, CG_ONEWIRE
  );
  TYPE clk_gen_en_t IS ARRAY (clk_gen_en_enum_t) OF STD_ULOGIC;
  SIGNAL clk_gen_en : clk_gen_en_t;
  SIGNAL clk_gen_en2 : STD_ULOGIC_VECTOR(10 DOWNTO 0);

  -- debug module interface (DMI) --
  SIGNAL dmi_req : dmi_req_t;
  SIGNAL dmi_rsp : dmi_rsp_t;

  -- debug core interface (DCI) --
  SIGNAL dci_ndmrstn : STD_ULOGIC;
  SIGNAL dci_haltreq : STD_ULOGIC_VECTOR(num_cores_c - 1 DOWNTO 0);

  -- CPU ICC links --
  TYPE core_complex_icc_t IS ARRAY (0 TO num_cores_c - 1) OF icc_t;
  SIGNAL icc_tx, icc_rx : core_complex_icc_t;

  -- bus: CPU core complex --
  TYPE core_complex_req_t IS ARRAY (0 TO num_cores_c - 1) OF bus_req_t;
  TYPE core_complex_rsp_t IS ARRAY (0 TO num_cores_c - 1) OF bus_rsp_t;
  SIGNAL cpu_i_req, cpu_d_req, icache_req, dcache_req, core_req : core_complex_req_t;
  SIGNAL cpu_i_rsp, cpu_d_rsp, icache_rsp, dcache_rsp, core_rsp : core_complex_rsp_t;

  -- bus: system bus (including DMA complex) --
  SIGNAL sys1_req, sys2_req, dma_req, amo_req, sys3_req : bus_req_t;
  SIGNAL sys1_rsp, sys2_rsp, dma_rsp, amo_rsp, sys3_rsp : bus_rsp_t;

  -- bus: main sections --
  SIGNAL imem_req, dmem_req, io_req, xcache_req, xbus_req : bus_req_t;
  SIGNAL imem_rsp, dmem_rsp, io_rsp, xcache_rsp, xbus_rsp : bus_rsp_t;

  -- bus: IO devices --
  TYPE io_devices_enum_t IS (
    IODEV_BOOTROM, IODEV_OCD, IODEV_SYSINFO, IODEV_NEOLED, IODEV_GPIO, IODEV_WDT, IODEV_TRNG,
    IODEV_TWI, IODEV_SPI, IODEV_SDI, IODEV_UART1, IODEV_UART0, IODEV_CLINT, IODEV_ONEWIRE,
    IODEV_GPTMR, IODEV_PWM, IODEV_CRC, IODEV_DMA, IODEV_SLINK, IODEV_CFS, IODEV_TWD
  );
  TYPE iodev_req_t IS ARRAY (io_devices_enum_t) OF bus_req_t;
  TYPE iodev_rsp_t IS ARRAY (io_devices_enum_t) OF bus_rsp_t;
  SIGNAL iodev_req : iodev_req_t;
  SIGNAL iodev_rsp : iodev_rsp_t;

  -- memory synchronization / ordering / coherence --
  SIGNAL mem_sync, dcache_clean : STD_ULOGIC_VECTOR(num_cores_c - 1 DOWNTO 0);
  SIGNAL xcache_clean : STD_ULOGIC;

  -- IRQs --
  TYPE firq_enum_t IS (
    FIRQ_TWD, FIRQ_UART0_RX, FIRQ_UART0_TX, FIRQ_UART1_RX, FIRQ_UART1_TX, FIRQ_SPI, FIRQ_SDI, FIRQ_TWI,
    FIRQ_CFS, FIRQ_NEOLED, FIRQ_GPIO, FIRQ_GPTMR, FIRQ_ONEWIRE, FIRQ_DMA, FIRQ_SLINK_RX, FIRQ_SLINK_TX
  );
  TYPE firq_t IS ARRAY (firq_enum_t) OF STD_ULOGIC;
  SIGNAL firq : firq_t;
  SIGNAL cpu_firq : STD_ULOGIC_VECTOR(15 DOWNTO 0);
  SIGNAL mtime_irq : STD_ULOGIC_VECTOR(num_cores_c - 1 DOWNTO 0);
  SIGNAL msw_irq : STD_ULOGIC_VECTOR(num_cores_c - 1 DOWNTO 0);

  -- Custom Signals HW Acc <--> SRAM
  SIGNAL data_sram_acc : STD_LOGIC_VECTOR(PE_COUNT * 16 - 1 DOWNTO 0);
  SIGNAL data_acc_sram : STD_LOGIC_VECTOR(PE_COUNT * 16 - 1 DOWNTO 0);
  SIGNAL rw_acc_sram : STD_LOGIC;
  SIGNAL ena_acc_sram : STD_LOGIC;
  SIGNAL addr_acc_sram : STD_LOGIC_VECTOR(24 - 1 DOWNTO 0);
BEGIN

  -- **************************************************************************************************************************
  -- Sanity Checks
  -- **************************************************************************************************************************

  sanity_checks :
  IF true GENERATE

    -- say hello --
    ASSERT false REPORT
    "[NEORV32] The NEORV32 RISC-V Processor " &
    "(v" & print_version_f(hw_version_c) & "), " &
    "github.com/stnolting/neorv32" SEVERITY note;

    -- show SoC configuration --
    ASSERT false REPORT
    "[NEORV32] Processor Configuration: CPU " & -- cpu core is always enabled
    cond_sel_string_f(BOOLEAN(num_cores_c = 1), "(single-core) ", "") &
    cond_sel_string_f(BOOLEAN(num_cores_c = 2), "(smp-dual-core) ", "") &
    cond_sel_string_f(MEM_INT_IMEM_EN, cond_sel_string_f(imem_as_rom_c, "IMEM-ROM ", "IMEM "), "") &
    cond_sel_string_f(MEM_INT_DMEM_EN, "DMEM ", "") &
    cond_sel_string_f(bootrom_en_c, "BOOTROM ", "") &
    cond_sel_string_f(ICACHE_EN, "I-CACHE ", "") &
    cond_sel_string_f(DCACHE_EN, "D-CACHE ", "") &
    cond_sel_string_f(XBUS_EN, "XBUS ", "") &
    cond_sel_string_f(XBUS_EN AND XBUS_CACHE_EN, "XBUS-CACHE ", "") &
    cond_sel_string_f(IO_CLINT_EN, "CLINT ", "") &
    cond_sel_string_f(io_gpio_en_c, "GPIO ", "") &
    cond_sel_string_f(IO_UART0_EN, "UART0 ", "") &
    cond_sel_string_f(IO_UART1_EN, "UART1 ", "") &
    cond_sel_string_f(IO_SPI_EN, "SPI ", "") &
    cond_sel_string_f(IO_SDI_EN, "SDI ", "") &
    cond_sel_string_f(IO_TWI_EN, "TWI ", "") &
    cond_sel_string_f(IO_TWD_EN, "TWD ", "") &
    cond_sel_string_f(io_pwm_en_c, "PWM ", "") &
    cond_sel_string_f(IO_WDT_EN, "WDT ", "") &
    cond_sel_string_f(IO_TRNG_EN, "TRNG ", "") &
    cond_sel_string_f(IO_CFS_EN, "CFS ", "") &
    cond_sel_string_f(IO_NEOLED_EN, "NEOLED ", "") &
    cond_sel_string_f(IO_GPTMR_EN, "GPTMR ", "") &
    cond_sel_string_f(IO_ONEWIRE_EN, "ONEWIRE ", "") &
    cond_sel_string_f(IO_DMA_EN, "DMA ", "") &
    cond_sel_string_f(IO_SLINK_EN, "SLINK ", "") &
    cond_sel_string_f(IO_CRC_EN, "CRC ", "") &
    cond_sel_string_f(io_sysinfo_en_c, "SYSINFO ", "") &
    cond_sel_string_f(OCD_EN, cond_sel_string_f(OCD_AUTHENTICATION, "OCD-AUTH ", "OCD "), "") &
    ""
    SEVERITY note;

    -- IMEM size was not a power of two --
    ASSERT NOT ((MEM_INT_IMEM_SIZE /= imem_size_c) AND (MEM_INT_IMEM_EN = true)) REPORT
    "[NEORV32] Auto-adjusting invalid IMEM size configuration." SEVERITY warning;

    -- DMEM size was not a power of two --
    ASSERT NOT ((MEM_INT_DMEM_SIZE /= dmem_size_c) AND (MEM_INT_DMEM_EN = true)) REPORT
    "[NEORV32] Auto-adjusting invalid DMEM size configuration." SEVERITY warning;

    -- SYSINFO disabled --
    ASSERT NOT (io_sysinfo_en_c = false) REPORT
    "[NEORV32] SYSINFO module disabled - some parts of the NEORV32 software framework will no longer work!" SEVERITY warning;

    -- Clock speed not defined --
    ASSERT NOT (CLOCK_FREQUENCY = 0) REPORT
    "[NEORV32] CLOCK_FREQUENCY must be configured according to the frequency of clk_i port." SEVERITY warning;

    -- Boot configuration notifier --
    ASSERT NOT (BOOT_MODE_SELECT = 0) REPORT "[NEORV32] BOOT_MODE_SELECT = 0: booting via bootloader" SEVERITY note;
    ASSERT NOT (BOOT_MODE_SELECT = 1) REPORT "[NEORV32] BOOT_MODE_SELECT = 1: booting from custom address" SEVERITY note;
    ASSERT NOT (BOOT_MODE_SELECT = 2) REPORT "[NEORV32] BOOT_MODE_SELECT = 2: booting IMEM image" SEVERITY note;

    -- Boot configuration: boot from initialized IMEM requires the IMEM to be enabled --
    ASSERT NOT ((BOOT_MODE_SELECT = 2) AND (MEM_INT_IMEM_EN = false)) REPORT
    "[NEORV32] ERROR: BOOT_MODE_SELECT = 2 (boot IMEM image) requires the internal instruction memory (IMEM) to be enabled!" SEVERITY error;

    -- The SMP dual-core configuration requires the CLINT --
    ASSERT NOT ((DUAL_CORE_EN = true) AND (IO_CLINT_EN = false)) REPORT
    "[NEORV32] ERROR: The SMP dual-core configuration requires the CLINT to be enabled!" SEVERITY error;

  END GENERATE; -- /sanity_checks
  -- **************************************************************************************************************************
  -- Clock and Reset Generators
  -- **************************************************************************************************************************

  soc_generators :
  IF true GENERATE

    -- Reset Sequencer ------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sys_reset_inst : ENTITY neorv32.neorv32_sys_reset
      PORT MAP(
        clk_i => clk_i,
        rstn_ext_i => rstn_i,
        rstn_wdt_i => rstn_wdt,
        rstn_dbg_i => dci_ndmrstn,
        rstn_ext_o => rstn_ext,
        rstn_sys_o => rstn_sys,
        xrstn_wdt_o => rstn_wdt_o,
        xrstn_ocd_o => rstn_ocd_o
      );
    -- Clock Divider / Pulse Generator --------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sys_clock_inst : ENTITY neorv32.neorv32_sys_clock
      GENERIC MAP(
        NUM_EN => clk_gen_en2'length
      )
      PORT MAP(
        clk_i => clk_i,
        rstn_i => rstn_sys,
        enable_i => clk_gen_en2,
        clk_en_o => clk_gen
      );

    -- fresh clocks anyone? --
    clk_gen_en2 <= clk_gen_en(CG_CFS) & clk_gen_en(CG_UART0) & clk_gen_en(CG_UART1) & clk_gen_en(CG_SPI) &
      clk_gen_en(CG_TWI) & clk_gen_en(CG_TWD) & clk_gen_en(CG_PWM) & clk_gen_en(CG_WDT) &
      clk_gen_en(CG_NEOLED) & clk_gen_en(CG_GPTMR) & clk_gen_en(CG_ONEWIRE);

  END GENERATE; -- /soc_generators
  -- **************************************************************************************************************************
  -- Core Complex
  -- **************************************************************************************************************************

  -- fast interrupt requests (FIRQs) --
  cpu_firq(0) <= firq(FIRQ_TWD); -- highest priority
  cpu_firq(1) <= firq(FIRQ_CFS);
  cpu_firq(2) <= firq(FIRQ_UART0_RX);
  cpu_firq(3) <= firq(FIRQ_UART0_TX);
  cpu_firq(4) <= firq(FIRQ_UART1_RX);
  cpu_firq(5) <= firq(FIRQ_UART1_TX);
  cpu_firq(6) <= firq(FIRQ_SPI);
  cpu_firq(7) <= firq(FIRQ_TWI);
  cpu_firq(8) <= firq(FIRQ_GPIO);
  cpu_firq(9) <= firq(FIRQ_NEOLED);
  cpu_firq(10) <= firq(FIRQ_DMA);
  cpu_firq(11) <= firq(FIRQ_SDI);
  cpu_firq(12) <= firq(FIRQ_GPTMR);
  cpu_firq(13) <= firq(FIRQ_ONEWIRE);
  cpu_firq(14) <= firq(FIRQ_SLINK_RX);
  cpu_firq(15) <= firq(FIRQ_SLINK_TX); -- lowest priority

  -- CPU core(s) + optional L1 caches + bus switch --
  core_complex_gen :
  FOR i IN 0 TO num_cores_c - 1 GENERATE

    -- CPU Core -------------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cpu_inst : ENTITY neorv32.neorv32_cpu
      GENERIC MAP(
        -- General --
        HART_ID => i + HART_BASE,
        BOOT_ADDR => cpu_boot_addr_c,
        DEBUG_PARK_ADDR => dm_park_entry_c,
        DEBUG_EXC_ADDR => dm_exc_entry_c,
        ICC_EN => DUAL_CORE_EN,
        -- RISC-V ISA Extensions --
        RISCV_ISA_C => RISCV_ISA_C,
        RISCV_ISA_E => RISCV_ISA_E,
        RISCV_ISA_M => RISCV_ISA_M,
        RISCV_ISA_U => RISCV_ISA_U,
        RISCV_ISA_Zaamo => RISCV_ISA_Zaamo,
        RISCV_ISA_Zalrsc => RISCV_ISA_Zalrsc,
        RISCV_ISA_Zba => RISCV_ISA_Zba,
        RISCV_ISA_Zbb => RISCV_ISA_Zbb,
        RISCV_ISA_Zbkb => RISCV_ISA_Zbkb,
        RISCV_ISA_Zbkc => RISCV_ISA_Zbkc,
        RISCV_ISA_Zbkx => RISCV_ISA_Zbkx,
        RISCV_ISA_Zbs => RISCV_ISA_Zbs,
        RISCV_ISA_Zfinx => RISCV_ISA_Zfinx,
        RISCV_ISA_Zicntr => RISCV_ISA_Zicntr,
        RISCV_ISA_Zicond => RISCV_ISA_Zicond,
        RISCV_ISA_Zihpm => RISCV_ISA_Zihpm,
        RISCV_ISA_Zknd => RISCV_ISA_Zknd,
        RISCV_ISA_Zkne => RISCV_ISA_Zkne,
        RISCV_ISA_Zknh => RISCV_ISA_Zknh,
        RISCV_ISA_Zksed => RISCV_ISA_Zksed,
        RISCV_ISA_Zksh => RISCV_ISA_Zksh,
        RISCV_ISA_Zmmul => RISCV_ISA_Zmmul,
        RISCV_ISA_Zxcfu => RISCV_ISA_Zxcfu,
        RISCV_ISA_Sdext => OCD_EN,
        RISCV_ISA_Sdtrig => OCD_EN,
        RISCV_ISA_Smpmp => cpu_smpmp_c,
        -- Tuning Options --
        CPU_FAST_MUL_EN => CPU_FAST_MUL_EN,
        CPU_FAST_SHIFT_EN => CPU_FAST_SHIFT_EN,
        CPU_RF_HW_RST_EN => CPU_RF_HW_RST_EN,
        -- Physical Memory Protection (PMP) --
        PMP_NUM_REGIONS => PMP_NUM_REGIONS,
        PMP_MIN_GRANULARITY => PMP_MIN_GRANULARITY,
        PMP_TOR_MODE_EN => PMP_TOR_MODE_EN,
        PMP_NAP_MODE_EN => PMP_NAP_MODE_EN,
        -- Hardware Performance Monitors (HPM) --
        HPM_NUM_CNTS => HPM_NUM_CNTS,
        HPM_CNT_WIDTH => HPM_CNT_WIDTH
      )
      PORT MAP(
        -- global control --
        clk_i => clk_i,
        rstn_i => rstn_sys,
        -- interrupts --
        msi_i => msw_irq(i),
        mei_i => mext_irq_i,
        mti_i => mtime_irq(i),
        firq_i => cpu_firq,
        dbi_i => dci_haltreq(i),
        -- inter-core communication links --
        icc_tx_o => icc_tx(i),
        icc_rx_i => icc_rx(i),
        -- instruction bus interface --
        ibus_req_o => cpu_i_req(i),
        ibus_rsp_i => cpu_i_rsp(i),
        -- data bus interface --
        dbus_req_o => cpu_d_req(i),
        dbus_rsp_i => cpu_d_rsp(i),
        -- memory synchronization --
        mem_sync_i => mem_sync(i)
      );

    -- memory synchronization (ordering / coherence) --
    mem_sync(i) <= dcache_clean(i) AND xcache_clean; -- for this hart's perspective only
    -- CPU L1 Instruction Cache ---------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_icache_enabled :
    IF ICACHE_EN GENERATE
      neorv32_icache_inst : ENTITY neorv32.neorv32_cache
        GENERIC MAP(
          NUM_BLOCKS => ICACHE_NUM_BLOCKS,
          BLOCK_SIZE => ICACHE_BLOCK_SIZE,
          UC_BEGIN => mem_uncached_begin_c(31 DOWNTO 28),
          READ_ONLY => true
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          clean_o => OPEN, -- cache is read-only so it cannot be dirty
          host_req_i => cpu_i_req(i),
          host_rsp_o => cpu_i_rsp(i),
          bus_req_o => icache_req(i),
          bus_rsp_i => icache_rsp(i)
        );
    END GENERATE;

    neorv32_icache_disabled :
    IF NOT ICACHE_EN GENERATE
      icache_req(i) <= cpu_i_req(i);
      cpu_i_rsp(i) <= icache_rsp(i);
    END GENERATE;
    -- CPU L1 Data Cache ----------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dcache_enabled :
    IF DCACHE_EN GENERATE
      neorv32_dcache_inst : ENTITY neorv32.neorv32_cache
        GENERIC MAP(
          NUM_BLOCKS => DCACHE_NUM_BLOCKS,
          BLOCK_SIZE => DCACHE_BLOCK_SIZE,
          UC_BEGIN => mem_uncached_begin_c(31 DOWNTO 28),
          READ_ONLY => false
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          clean_o => dcache_clean(i),
          host_req_i => cpu_d_req(i),
          host_rsp_o => cpu_d_rsp(i),
          bus_req_o => dcache_req(i),
          bus_rsp_i => dcache_rsp(i)
        );
    END GENERATE;

    neorv32_dcache_disabled :
    IF NOT DCACHE_EN GENERATE
      dcache_clean(i) <= '1';
      dcache_req(i) <= cpu_d_req(i);
      cpu_d_rsp(i) <= dcache_rsp(i);
    END GENERATE;
    -- Core Instruction/Data Bus Switch -------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_core_bus_switch_inst : ENTITY neorv32.neorv32_bus_switch
      GENERIC MAP(
        ROUND_ROBIN_EN => false, -- use prioritizing arbitration
        PORT_A_READ_ONLY => false,
        PORT_B_READ_ONLY => true -- instruction fetch is read-only
      )
      PORT MAP(
        clk_i => clk_i,
        rstn_i => rstn_sys,
        a_req_i => dcache_req(i), -- data accesses are prioritized
        a_rsp_o => dcache_rsp(i),
        b_req_i => icache_req(i),
        b_rsp_o => icache_rsp(i),
        x_req_o => core_req(i),
        x_rsp_i => core_rsp(i)
      );

  END GENERATE; -- /core_complex
  -- Inter-Core Communication (ICC) Links ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  icc_connect : PROCESS (icc_tx)
  BEGIN
    icc_rx(icc_rx'left) <= icc_tx(icc_tx'right);
    icc_rx(icc_rx'right) <= icc_tx(icc_tx'left);
  END PROCESS icc_connect;
  -- Core Complex Bus Arbiter ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  core_complex_dual :
  IF num_cores_c = 2 GENERATE
    neorv32_complex_arbiter_inst : ENTITY neorv32.neorv32_bus_switch
      GENERIC MAP(
        ROUND_ROBIN_EN => true,
        PORT_A_READ_ONLY => false,
        PORT_B_READ_ONLY => false
      )
      PORT MAP(
        clk_i => clk_i,
        rstn_i => rstn_sys,
        a_req_i => core_req(core_req'left),
        a_rsp_o => core_rsp(core_rsp'left),
        b_req_i => core_req(core_req'right),
        b_rsp_o => core_rsp(core_rsp'right),
        x_req_o => sys1_req,
        x_rsp_i => sys1_rsp
      );
  END GENERATE;

  core_complex_single :
  IF num_cores_c = 1 GENERATE
    sys1_req <= core_req(0);
    core_rsp(0) <= sys1_rsp;
  END GENERATE;
  -- **************************************************************************************************************************
  -- Direct Memory Access Controller (DMA) Complex
  -- **************************************************************************************************************************

  neorv32_dma_complex_enabled :
  IF IO_DMA_EN GENERATE

    -- DMA Controller -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dma_inst : ENTITY neorv32.neorv32_dma
      PORT MAP(
        clk_i => clk_i,
        rstn_i => rstn_sys,
        bus_req_i => iodev_req(IODEV_DMA),
        bus_rsp_o => iodev_rsp(IODEV_DMA),
        dma_req_o => dma_req,
        dma_rsp_i => dma_rsp,
        irq_o => firq(FIRQ_DMA)
      );
    -- DMA Bus Switch -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dma_bus_switch_inst : ENTITY neorv32.neorv32_bus_switch
      GENERIC MAP(
        ROUND_ROBIN_EN => false, -- use prioritizing arbitration
        PORT_A_READ_ONLY => false,
        PORT_B_READ_ONLY => false
      )
      PORT MAP(
        clk_i => clk_i,
        rstn_i => rstn_sys,
        a_req_i => sys1_req, -- prioritized
        a_rsp_o => sys1_rsp,
        b_req_i => dma_req,
        b_rsp_o => dma_rsp,
        x_req_o => sys2_req,
        x_rsp_i => sys2_rsp
      );

  END GENERATE; -- /neorv32_dma_complex_enabled

  neorv32_dma_complex_disabled :
  IF NOT IO_DMA_EN GENERATE
    iodev_rsp(IODEV_DMA) <= rsp_terminate_c;
    sys2_req <= sys1_req;
    sys1_rsp <= sys2_rsp;
    firq(FIRQ_DMA) <= '0';
  END GENERATE;
  -- **************************************************************************************************************************
  -- Atomic Memory Operations
  -- **************************************************************************************************************************

  atomics :
  IF true GENERATE

    -- Read-Modify-Write Controller -----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_bus_amo_rmw_enabled :
    IF RISCV_ISA_Zaamo GENERATE
      neorv32_bus_amo_rmw_inst : ENTITY neorv32.neorv32_bus_amo_rmw
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          core_req_i => sys2_req,
          core_rsp_o => sys2_rsp,
          sys_req_o => amo_req,
          sys_rsp_i => amo_rsp
        );
    END GENERATE;

    neorv32_bus_amo_rmw_disabled :
    IF NOT RISCV_ISA_Zaamo GENERATE
      amo_req <= sys2_req;
      sys2_rsp <= amo_rsp;
    END GENERATE;
    -- Reservation-Set Controller -------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_bus_amo_rvs_enabled :
    IF RISCV_ISA_Zalrsc GENERATE
      neorv32_bus_amo_rvs_inst : ENTITY neorv32.neorv32_bus_amo_rvs
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          core_req_i => amo_req,
          core_rsp_o => amo_rsp,
          sys_req_o => sys3_req,
          sys_rsp_i => sys3_rsp
        );
    END GENERATE;

    neorv32_bus_amo_rvs_disabled :
    IF NOT RISCV_ISA_Zalrsc GENERATE
      sys3_req <= amo_req;
      amo_rsp <= sys3_rsp;
    END GENERATE;

  END GENERATE; -- /atomics
  -- **************************************************************************************************************************
  -- Address Region Gateway
  -- **************************************************************************************************************************

  neorv32_bus_gateway_inst : ENTITY neorv32.neorv32_bus_gateway
    GENERIC MAP(
      TIMEOUT => bus_timeout_c,
      -- port A: internal IMEM --
      A_EN => MEM_INT_IMEM_EN,
      A_BASE => mem_imem_base_c,
      A_SIZE => imem_size_c,
      -- port B: internal DMEM --
      B_EN => MEM_INT_DMEM_EN,
      B_BASE => mem_dmem_base_c,
      B_SIZE => dmem_size_c,
      -- port C: IO --
      C_EN => true, -- always enabled (but will be trimmed if no IO devices are implemented)
      C_BASE => mem_io_base_c,
      C_SIZE => mem_io_size_c,
      -- port X (the void): XBUS --
      X_EN => XBUS_EN
    )
    PORT MAP(
      -- global control --
      clk_i => clk_i,
      rstn_i => rstn_sys,
      -- host port --
      req_i => sys3_req,
      rsp_o => sys3_rsp,
      -- section ports --
      a_req_o => imem_req,
      a_rsp_i => imem_rsp,
      b_req_o => dmem_req,
      b_rsp_i => dmem_rsp,
      c_req_o => io_req,
      c_rsp_i => io_rsp,
      x_req_o => xbus_req,
      x_rsp_i => xbus_rsp
    );
  -- **************************************************************************************************************************
  -- Memory System
  -- **************************************************************************************************************************

  memory_system :
  IF true GENERATE

    -- Processor-Internal Instruction Memory (IMEM) -------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_int_imem_enabled :
    IF MEM_INT_IMEM_EN GENERATE
      neorv32_int_imem_inst : ENTITY neorv32.neorv32_imem
        GENERIC MAP(
          IMEM_SIZE => imem_size_c,
          IMEM_INIT => imem_as_rom_c
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => imem_req,
          bus_rsp_o => imem_rsp
        );
    END GENERATE;

    neorv32_int_imem_disabled :
    IF NOT MEM_INT_IMEM_EN GENERATE
      imem_rsp <= rsp_terminate_c;
    END GENERATE;
    -- Processor-Internal Data Memory (DMEM) --------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_int_dmem_inst_true :
    IF MEM_INT_DMEM_EN GENERATE
      neorv32_int_dmem_inst : ENTITY neorv32.neorv32_dmem_tc
        GENERIC MAP(
          DMEM_SIZE => dmem_size_c,
          ACC_DATA_WIDTH => 16,
          PE_COUNT => PE_COUNT
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => dmem_req,
          bus_rsp_o => dmem_rsp,
          acc_data_in => data_acc_sram,
          acc_ena => ena_acc_sram,
          acc_rw => rw_acc_sram,
          acc_addr => addr_acc_sram,
          acc_data_out => data_sram_acc
        );
    END GENERATE;

    neorv32_int_dmem_disabled :
    IF NOT MEM_INT_DMEM_EN GENERATE
      dmem_rsp <= rsp_terminate_c;
    END GENERATE;
    -- External Bus Interface (XBUS) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_xbus_enabled :
    IF XBUS_EN GENERATE

      -- external bus gateway (XBUS) --
      neorv32_xbus_inst : ENTITY neorv32.neorv32_xbus
        GENERIC MAP(
          TIMEOUT_VAL => XBUS_TIMEOUT,
          REGSTAGE_EN => XBUS_REGSTAGE_EN
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => xcache_req,
          bus_rsp_o => xcache_rsp,
          xbus_adr_o => xbus_adr_o,
          xbus_dat_i => xbus_dat_i,
          xbus_dat_o => xbus_dat_o,
          xbus_tag_o => xbus_tag_o,
          xbus_we_o => xbus_we_o,
          xbus_sel_o => xbus_sel_o,
          xbus_stb_o => xbus_stb_o,
          xbus_cyc_o => xbus_cyc_o,
          xbus_ack_i => xbus_ack_i,
          xbus_err_i => xbus_err_i
        );

      -- external bus cache (X-CACHE) --
      neorv32_xcache_enabled :
      IF XBUS_CACHE_EN GENERATE
        neorv32_xcache_inst : ENTITY neorv32.neorv32_cache
          GENERIC MAP(
            NUM_BLOCKS => XBUS_CACHE_NUM_BLOCKS,
            BLOCK_SIZE => XBUS_CACHE_BLOCK_SIZE,
            UC_BEGIN => mem_uncached_begin_c(31 DOWNTO 28),
            READ_ONLY => false
          )
          PORT MAP(
            clk_i => clk_i,
            rstn_i => rstn_sys,
            clean_o => xcache_clean,
            host_req_i => xbus_req,
            host_rsp_o => xbus_rsp,
            bus_req_o => xcache_req,
            bus_rsp_i => xcache_rsp
          );
      END GENERATE;

      neorv32_xcache_disabled :
      IF NOT XBUS_CACHE_EN GENERATE
        xcache_clean <= '1';
        xcache_req <= xbus_req;
        xbus_rsp <= xcache_rsp;
      END GENERATE;

    END GENERATE; -- /neorv32_xbus_enabled

    neorv32_xbus_disabled :
    IF NOT XBUS_EN GENERATE
      xcache_clean <= '1';
      xcache_req <= req_terminate_c;
      xbus_rsp <= rsp_terminate_c;
      xbus_adr_o <= (OTHERS => '0');
      xbus_dat_o <= (OTHERS => '0');
      xbus_tag_o <= (OTHERS => '0');
      xbus_we_o <= '0';
      xbus_sel_o <= (OTHERS => '0');
      xbus_stb_o <= '0';
      xbus_cyc_o <= '0';
    END GENERATE;

  END GENERATE; -- /memory_system
  -- **************************************************************************************************************************
  -- IO/Peripheral Modules
  -- **************************************************************************************************************************

  io_system :
  IF true GENERATE

    -- IO Switch ------------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_bus_io_switch_inst : ENTITY neorv32.neorv32_bus_io_switch
      GENERIC MAP(
        INREG_EN => true,
        OUTREG_EN => true,
        DEV_SIZE => iodev_size_c,
        DEV_00_EN => bootrom_en_c, DEV_00_BASE => base_io_bootrom_c,
        DEV_01_EN => false, DEV_01_BASE => (OTHERS => '0'), -- reserved
        DEV_02_EN => false, DEV_02_BASE => (OTHERS => '0'), -- reserved
        DEV_03_EN => false, DEV_03_BASE => (OTHERS => '0'), -- reserved
        DEV_04_EN => false, DEV_04_BASE => (OTHERS => '0'), -- reserved
        DEV_05_EN => false, DEV_05_BASE => (OTHERS => '0'), -- reserved
        DEV_06_EN => false, DEV_06_BASE => (OTHERS => '0'), -- reserved
        DEV_07_EN => false, DEV_07_BASE => (OTHERS => '0'), -- reserved
        DEV_08_EN => false, DEV_08_BASE => (OTHERS => '0'), -- reserved
        DEV_09_EN => false, DEV_09_BASE => (OTHERS => '0'), -- reserved
        DEV_10_EN => IO_TWD_EN, DEV_10_BASE => base_io_twd_c,
        DEV_11_EN => IO_CFS_EN, DEV_11_BASE => base_io_cfs_c,
        DEV_12_EN => IO_SLINK_EN, DEV_12_BASE => base_io_slink_c,
        DEV_13_EN => IO_DMA_EN, DEV_13_BASE => base_io_dma_c,
        DEV_14_EN => IO_CRC_EN, DEV_14_BASE => base_io_crc_c,
        DEV_15_EN => false, DEV_15_BASE => (OTHERS => '0'), -- reserved
        DEV_16_EN => io_pwm_en_c, DEV_16_BASE => base_io_pwm_c,
        DEV_17_EN => IO_GPTMR_EN, DEV_17_BASE => base_io_gptmr_c,
        DEV_18_EN => IO_ONEWIRE_EN, DEV_18_BASE => base_io_onewire_c,
        DEV_19_EN => false, DEV_19_BASE => (OTHERS => '0'), -- reserved
        DEV_20_EN => IO_CLINT_EN, DEV_20_BASE => base_io_clint_c,
        DEV_21_EN => IO_UART0_EN, DEV_21_BASE => base_io_uart0_c,
        DEV_22_EN => IO_UART1_EN, DEV_22_BASE => base_io_uart1_c,
        DEV_23_EN => IO_SDI_EN, DEV_23_BASE => base_io_sdi_c,
        DEV_24_EN => IO_SPI_EN, DEV_24_BASE => base_io_spi_c,
        DEV_25_EN => IO_TWI_EN, DEV_25_BASE => base_io_twi_c,
        DEV_26_EN => IO_TRNG_EN, DEV_26_BASE => base_io_trng_c,
        DEV_27_EN => IO_WDT_EN, DEV_27_BASE => base_io_wdt_c,
        DEV_28_EN => io_gpio_en_c, DEV_28_BASE => base_io_gpio_c,
        DEV_29_EN => IO_NEOLED_EN, DEV_29_BASE => base_io_neoled_c,
        DEV_30_EN => io_sysinfo_en_c, DEV_30_BASE => base_io_sysinfo_c,
        DEV_31_EN => OCD_EN, DEV_31_BASE => base_io_ocd_c
      )
      PORT MAP(
        clk_i => clk_i,
        rstn_i => rstn_sys,
        main_req_i => io_req,
        main_rsp_o => io_rsp,
        dev_00_req_o => iodev_req(IODEV_BOOTROM), dev_00_rsp_i => iodev_rsp(IODEV_BOOTROM),
        dev_01_req_o => OPEN, dev_01_rsp_i => rsp_terminate_c, -- reserved
        dev_02_req_o => OPEN, dev_02_rsp_i => rsp_terminate_c, -- reserved
        dev_03_req_o => OPEN, dev_03_rsp_i => rsp_terminate_c, -- reserved
        dev_04_req_o => OPEN, dev_04_rsp_i => rsp_terminate_c, -- reserved
        dev_05_req_o => OPEN, dev_05_rsp_i => rsp_terminate_c, -- reserved
        dev_06_req_o => OPEN, dev_06_rsp_i => rsp_terminate_c, -- reserved
        dev_07_req_o => OPEN, dev_07_rsp_i => rsp_terminate_c, -- reserved
        dev_08_req_o => OPEN, dev_08_rsp_i => rsp_terminate_c, -- reserved
        dev_09_req_o => OPEN, dev_09_rsp_i => rsp_terminate_c, -- reserved
        dev_10_req_o => iodev_req(IODEV_TWD), dev_10_rsp_i => iodev_rsp(IODEV_TWD),
        dev_11_req_o => iodev_req(IODEV_CFS), dev_11_rsp_i => iodev_rsp(IODEV_CFS),
        dev_12_req_o => iodev_req(IODEV_SLINK), dev_12_rsp_i => iodev_rsp(IODEV_SLINK),
        dev_13_req_o => iodev_req(IODEV_DMA), dev_13_rsp_i => iodev_rsp(IODEV_DMA),
        dev_14_req_o => iodev_req(IODEV_CRC), dev_14_rsp_i => iodev_rsp(IODEV_CRC),
        dev_15_req_o => OPEN, dev_15_rsp_i => rsp_terminate_c, -- reserved
        dev_16_req_o => iodev_req(IODEV_PWM), dev_16_rsp_i => iodev_rsp(IODEV_PWM),
        dev_17_req_o => iodev_req(IODEV_GPTMR), dev_17_rsp_i => iodev_rsp(IODEV_GPTMR),
        dev_18_req_o => iodev_req(IODEV_ONEWIRE), dev_18_rsp_i => iodev_rsp(IODEV_ONEWIRE),
        dev_19_req_o => OPEN, dev_19_rsp_i => rsp_terminate_c, -- reserved
        dev_20_req_o => iodev_req(IODEV_CLINT), dev_20_rsp_i => iodev_rsp(IODEV_CLINT),
        dev_21_req_o => iodev_req(IODEV_UART0), dev_21_rsp_i => iodev_rsp(IODEV_UART0),
        dev_22_req_o => iodev_req(IODEV_UART1), dev_22_rsp_i => iodev_rsp(IODEV_UART1),
        dev_23_req_o => iodev_req(IODEV_SDI), dev_23_rsp_i => iodev_rsp(IODEV_SDI),
        dev_24_req_o => iodev_req(IODEV_SPI), dev_24_rsp_i => iodev_rsp(IODEV_SPI),
        dev_25_req_o => iodev_req(IODEV_TWI), dev_25_rsp_i => iodev_rsp(IODEV_TWI),
        dev_26_req_o => iodev_req(IODEV_TRNG), dev_26_rsp_i => iodev_rsp(IODEV_TRNG),
        dev_27_req_o => iodev_req(IODEV_WDT), dev_27_rsp_i => iodev_rsp(IODEV_WDT),
        dev_28_req_o => iodev_req(IODEV_GPIO), dev_28_rsp_i => iodev_rsp(IODEV_GPIO),
        dev_29_req_o => iodev_req(IODEV_NEOLED), dev_29_rsp_i => iodev_rsp(IODEV_NEOLED),
        dev_30_req_o => iodev_req(IODEV_SYSINFO), dev_30_rsp_i => iodev_rsp(IODEV_SYSINFO),
        dev_31_req_o => iodev_req(IODEV_OCD), dev_31_rsp_i => iodev_rsp(IODEV_OCD)
      );
    -- Processor-Internal Bootloader ROM (BOOTROM) --------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_boot_rom_enabled :
    IF bootrom_en_c GENERATE
      neorv32_boot_rom_inst : ENTITY neorv32.neorv32_boot_rom
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_BOOTROM),
          bus_rsp_o => iodev_rsp(IODEV_BOOTROM)
        );
    END GENERATE;

    neorv32_boot_rom_disabled :
    IF NOT bootrom_en_c GENERATE
      iodev_rsp(IODEV_BOOTROM) <= rsp_terminate_c;
    END GENERATE;
    -- Custom Functions Subsystem (CFS) -------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cfs_inst_true :
    IF IO_CFS_EN GENERATE
      neorv32_cfs_inst : ENTITY neorv32.hw_acc_top
        GENERIC MAP(
          NUM_REGS_16b => 8,
          NUM_REGS_4b => 8,
          DATA_WIDTH => 16,
          PE_COUNT => PE_COUNT
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_CFS),
          bus_rsp_o => iodev_rsp(IODEV_CFS),
          irq_o => firq(FIRQ_CFS),
          sram_data_in => data_sram_acc,
          sram_data_out => data_acc_sram,
          sram_rw => rw_acc_sram,
          sram_ena => ena_acc_sram,
          sram_addr => addr_acc_sram
        );
    END GENERATE;

    neorv32_cfs_disabled :
    IF NOT IO_CFS_EN GENERATE
      iodev_rsp(IODEV_CFS) <= rsp_terminate_c;
      clk_gen_en(CG_CFS) <= '0';
      firq(FIRQ_CFS) <= '0';
      cfs_out_o <= (OTHERS => '0');
      addr_acc_sram <= (OTHERS => '0');
      ena_acc_sram <= '0';
      rw_acc_sram <= '0';
      data_acc_sram <= (OTHERS => '0');
    END GENERATE;
    -- Serial Data Interface (SDI) ------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sdi_enabled :
    IF IO_SDI_EN GENERATE
      neorv32_sdi_inst : ENTITY neorv32.neorv32_sdi
        GENERIC MAP(
          RTX_FIFO => IO_SDI_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_SDI),
          bus_rsp_o => iodev_rsp(IODEV_SDI),
          sdi_csn_i => sdi_csn_i,
          sdi_clk_i => sdi_clk_i,
          sdi_dat_i => sdi_dat_i,
          sdi_dat_o => sdi_dat_o,
          irq_o => firq(FIRQ_SDI)
        );
    END GENERATE;

    neorv32_sdi_disabled :
    IF NOT IO_SDI_EN GENERATE
      iodev_rsp(IODEV_SDI) <= rsp_terminate_c;
      sdi_dat_o <= '0';
      firq(FIRQ_SDI) <= '0';
    END GENERATE;
    -- General Purpose Input/Output Port (GPIO) -----------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_gpio_enabled :
    IF io_gpio_en_c GENERATE
      neorv32_gpio_inst : ENTITY neorv32.neorv32_gpio
        GENERIC MAP(
          GPIO_NUM => IO_GPIO_NUM
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_GPIO),
          bus_rsp_o => iodev_rsp(IODEV_GPIO),
          gpio_o => gpio_o,
          gpio_i => gpio_i,
          cpu_irq_o => firq(FIRQ_GPIO)
        );
    END GENERATE;

    neorv32_gpio_disabled :
    IF NOT io_gpio_en_c GENERATE
      iodev_rsp(IODEV_GPIO) <= rsp_terminate_c;
      gpio_o <= (OTHERS => '0');
      firq(FIRQ_GPIO) <= '0';
    END GENERATE;
    -- Watch Dog Timer (WDT) ------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_wdt_enabled :
    IF IO_WDT_EN GENERATE
      neorv32_wdt_inst : ENTITY neorv32.neorv32_wdt
        PORT MAP(
          clk_i => clk_i,
          rstn_ext_i => rstn_ext,
          rstn_dbg_i => dci_ndmrstn,
          rstn_sys_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_WDT),
          bus_rsp_o => iodev_rsp(IODEV_WDT),
          clkgen_en_o => clk_gen_en(CG_WDT),
          clkgen_i => clk_gen,
          rstn_o => rstn_wdt
        );
    END GENERATE;

    neorv32_wdt_disabled :
    IF NOT IO_WDT_EN GENERATE
      iodev_rsp(IODEV_WDT) <= rsp_terminate_c;
      clk_gen_en(CG_WDT) <= '0';
      rstn_wdt <= '1';
    END GENERATE;
    -- Core Local Interruptor (CLINT) ---------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_clint_enabled :
    IF IO_CLINT_EN GENERATE
      neorv32_clint_inst : ENTITY neorv32.neorv32_clint
        GENERIC MAP(
          NUM_HARTS => num_cores_c
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_CLINT),
          bus_rsp_o => iodev_rsp(IODEV_CLINT),
          time_o => mtime_time_o,
          mti_o => mtime_irq,
          msi_o => msw_irq
        );
    END GENERATE;

    neorv32_clint_disabled :
    IF NOT IO_CLINT_EN GENERATE
      iodev_rsp(IODEV_CLINT) <= rsp_terminate_c;
      mtime_time_o <= (OTHERS => '0');
      mtime_irq <= (OTHERS => mtime_irq_i);
      msw_irq <= (OTHERS => msw_irq_i);
    END GENERATE;
    -- Primary Universal Asynchronous Receiver/Transmitter (UART0) ----------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_uart0_enabled :
    IF IO_UART0_EN GENERATE
      neorv32_uart0_inst : ENTITY neorv32.neorv32_uart
        GENERIC MAP(
          SIM_MODE_EN => true,
          SIM_LOG_FILE => "neorv32.uart0_sim_mode.out",
          UART_RX_FIFO => IO_UART0_RX_FIFO,
          UART_TX_FIFO => IO_UART0_TX_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_UART0),
          bus_rsp_o => iodev_rsp(IODEV_UART0),
          clkgen_en_o => clk_gen_en(CG_UART0),
          clkgen_i => clk_gen,
          uart_txd_o => uart0_txd_o,
          uart_rxd_i => uart0_rxd_i,
          uart_rtsn_o => uart0_rtsn_o,
          uart_ctsn_i => uart0_ctsn_i,
          irq_rx_o => firq(FIRQ_UART0_RX),
          irq_tx_o => firq(FIRQ_UART0_TX)
        );
    END GENERATE;

    neorv32_uart0_disabled :
    IF NOT IO_UART0_EN GENERATE
      iodev_rsp(IODEV_UART0) <= rsp_terminate_c;
      uart0_txd_o <= '0';
      uart0_rtsn_o <= '1';
      clk_gen_en(CG_UART0) <= '0';
      firq(FIRQ_UART0_RX) <= '0';
      firq(FIRQ_UART0_TX) <= '0';
    END GENERATE;
    -- Secondary Universal Asynchronous Receiver/Transmitter (UART1) --------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_uart1_enabled :
    IF IO_UART1_EN GENERATE
      neorv32_uart1_inst : ENTITY neorv32.neorv32_uart
        GENERIC MAP(
          SIM_MODE_EN => true,
          SIM_LOG_FILE => "neorv32.uart1_sim_mode.out",
          UART_RX_FIFO => IO_UART1_RX_FIFO,
          UART_TX_FIFO => IO_UART1_TX_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_UART1),
          bus_rsp_o => iodev_rsp(IODEV_UART1),
          clkgen_en_o => clk_gen_en(CG_UART1),
          clkgen_i => clk_gen,
          uart_txd_o => uart1_txd_o,
          uart_rxd_i => uart1_rxd_i,
          uart_rtsn_o => uart1_rtsn_o,
          uart_ctsn_i => uart1_ctsn_i,
          irq_rx_o => firq(FIRQ_UART1_RX),
          irq_tx_o => firq(FIRQ_UART1_TX)
        );
    END GENERATE;

    neorv32_uart1_disabled :
    IF NOT IO_UART1_EN GENERATE
      iodev_rsp(IODEV_UART1) <= rsp_terminate_c;
      uart1_txd_o <= '0';
      uart1_rtsn_o <= '1';
      clk_gen_en(CG_UART1) <= '0';
      firq(FIRQ_UART1_RX) <= '0';
      firq(FIRQ_UART1_TX) <= '0';
    END GENERATE;
    -- Serial Peripheral Interface (SPI) ------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_spi_enabled :
    IF IO_SPI_EN GENERATE
      neorv32_spi_inst : ENTITY neorv32.neorv32_spi
        GENERIC MAP(
          IO_SPI_FIFO => IO_SPI_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_SPI),
          bus_rsp_o => iodev_rsp(IODEV_SPI),
          clkgen_en_o => clk_gen_en(CG_SPI),
          clkgen_i => clk_gen,
          spi_clk_o => spi_clk_o,
          spi_dat_o => spi_dat_o,
          spi_dat_i => spi_dat_i,
          spi_csn_o => spi_csn_o,
          irq_o => firq(FIRQ_SPI)
        );
    END GENERATE;

    neorv32_spi_disabled :
    IF NOT IO_SPI_EN GENERATE
      iodev_rsp(IODEV_SPI) <= rsp_terminate_c;
      spi_clk_o <= '0';
      spi_dat_o <= '0';
      spi_csn_o <= (OTHERS => '1');
      clk_gen_en(CG_SPI) <= '0';
      firq(FIRQ_SPI) <= '0';
    END GENERATE;
    -- Two-Wire Interface (TWI) ---------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_twi_enabled :
    IF IO_TWI_EN GENERATE
      neorv32_twi_inst : ENTITY neorv32.neorv32_twi
        GENERIC MAP(
          IO_TWI_FIFO => IO_TWI_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_TWI),
          bus_rsp_o => iodev_rsp(IODEV_TWI),
          clkgen_en_o => clk_gen_en(CG_TWI),
          clkgen_i => clk_gen,
          twi_sda_i => twi_sda_i,
          twi_sda_o => twi_sda_o,
          twi_scl_i => twi_scl_i,
          twi_scl_o => twi_scl_o,
          irq_o => firq(FIRQ_TWI)
        );
    END GENERATE;

    neorv32_twi_disabled :
    IF NOT IO_TWI_EN GENERATE
      iodev_rsp(IODEV_TWI) <= rsp_terminate_c;
      twi_sda_o <= '1';
      twi_scl_o <= '1';
      clk_gen_en(CG_TWI) <= '0';
      firq(FIRQ_TWI) <= '0';
    END GENERATE;
    -- Two-Wire Device (TWD) ------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_twd_enabled :
    IF IO_TWD_EN GENERATE
      neorv32_twd_inst : ENTITY neorv32.neorv32_twd
        GENERIC MAP(
          TWD_FIFO => IO_TWD_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_TWD),
          bus_rsp_o => iodev_rsp(IODEV_TWD),
          clkgen_en_o => clk_gen_en(CG_TWD),
          clkgen_i => clk_gen,
          twd_sda_i => twd_sda_i,
          twd_sda_o => twd_sda_o,
          twd_scl_i => twd_scl_i,
          twd_scl_o => twd_scl_o,
          irq_o => firq(FIRQ_TWD)
        );
    END GENERATE;

    neorv32_twd_disabled :
    IF NOT IO_TWD_EN GENERATE
      iodev_rsp(IODEV_TWD) <= rsp_terminate_c;
      twd_sda_o <= '1';
      twd_scl_o <= '1';
      clk_gen_en(CG_TWD) <= '0';
      firq(FIRQ_TWD) <= '0';
    END GENERATE;
    -- Pulse-Width Modulation Controller (PWM) ------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_pwm_enabled :
    IF io_pwm_en_c GENERATE
      neorv32_pwm_inst : ENTITY neorv32.neorv32_pwm
        GENERIC MAP(
          NUM_CHANNELS => IO_PWM_NUM_CH
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_PWM),
          bus_rsp_o => iodev_rsp(IODEV_PWM),
          clkgen_en_o => clk_gen_en(CG_PWM),
          clkgen_i => clk_gen,
          pwm_o => pwm_o
        );
    END GENERATE;

    neorv32_pwm_disabled :
    IF NOT io_pwm_en_c GENERATE
      iodev_rsp(IODEV_PWM) <= rsp_terminate_c;
      clk_gen_en(CG_PWM) <= '0';
      pwm_o <= (OTHERS => '0');
    END GENERATE;
    -- True Random Number Generator (TRNG) ----------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_trng_enabled :
    IF IO_TRNG_EN GENERATE
      neorv32_trng_inst : ENTITY neorv32.neorv32_trng
        GENERIC MAP(
          TRNG_FIFO => IO_TRNG_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_TRNG),
          bus_rsp_o => iodev_rsp(IODEV_TRNG)
        );
    END GENERATE;

    neorv32_trng_disabled :
    IF NOT IO_TRNG_EN GENERATE
      iodev_rsp(IODEV_TRNG) <= rsp_terminate_c;
    END GENERATE;
    -- Smart LED (WS2811/WS2812) Interface (NEOLED) -------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_neoled_enabled :
    IF IO_NEOLED_EN GENERATE
      neorv32_neoled_inst : ENTITY neorv32.neorv32_neoled
        GENERIC MAP(
          FIFO_DEPTH => IO_NEOLED_TX_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_NEOLED),
          bus_rsp_o => iodev_rsp(IODEV_NEOLED),
          clkgen_en_o => clk_gen_en(CG_NEOLED),
          clkgen_i => clk_gen,
          irq_o => firq(FIRQ_NEOLED),
          neoled_o => neoled_o
        );
    END GENERATE;

    neorv32_neoled_disabled :
    IF NOT IO_NEOLED_EN GENERATE
      iodev_rsp(IODEV_NEOLED) <= rsp_terminate_c;
      clk_gen_en(CG_NEOLED) <= '0';
      firq(FIRQ_NEOLED) <= '0';
      neoled_o <= '0';
    END GENERATE;
    -- General Purpose Timer (GPTMR) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_gptmr_enabled :
    IF IO_GPTMR_EN GENERATE
      neorv32_gptmr_inst : ENTITY neorv32.neorv32_gptmr
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_GPTMR),
          bus_rsp_o => iodev_rsp(IODEV_GPTMR),
          clkgen_en_o => clk_gen_en(CG_GPTMR),
          clkgen_i => clk_gen,
          irq_o => firq(FIRQ_GPTMR)
        );
    END GENERATE;

    neorv32_gptmr_disabled :
    IF NOT IO_GPTMR_EN GENERATE
      iodev_rsp(IODEV_GPTMR) <= rsp_terminate_c;
      clk_gen_en(CG_GPTMR) <= '0';
      firq(FIRQ_GPTMR) <= '0';
    END GENERATE;
    -- 1-Wire Interface Controller (ONEWIRE) --------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_onewire_enabled :
    IF IO_ONEWIRE_EN GENERATE
      neorv32_onewire_inst : ENTITY neorv32.neorv32_onewire
        GENERIC MAP(
          ONEWIRE_FIFO => IO_ONEWIRE_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_ONEWIRE),
          bus_rsp_o => iodev_rsp(IODEV_ONEWIRE),
          clkgen_en_o => clk_gen_en(CG_ONEWIRE),
          clkgen_i => clk_gen,
          onewire_i => onewire_i,
          onewire_o => onewire_o,
          irq_o => firq(FIRQ_ONEWIRE)
        );
    END GENERATE;

    neorv32_onewire_disabled :
    IF NOT IO_ONEWIRE_EN GENERATE
      iodev_rsp(IODEV_ONEWIRE) <= rsp_terminate_c;
      onewire_o <= '1';
      clk_gen_en(CG_ONEWIRE) <= '0';
      firq(FIRQ_ONEWIRE) <= '0';
    END GENERATE;
    -- Stream Link Interface (SLINK) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_slink_enabled :
    IF IO_SLINK_EN GENERATE
      neorv32_slink_inst : ENTITY neorv32.neorv32_slink
        GENERIC MAP(
          SLINK_RX_FIFO => IO_SLINK_RX_FIFO,
          SLINK_TX_FIFO => IO_SLINK_TX_FIFO
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_SLINK),
          bus_rsp_o => iodev_rsp(IODEV_SLINK),
          rx_irq_o => firq(FIRQ_SLINK_RX),
          tx_irq_o => firq(FIRQ_SLINK_TX),
          slink_rx_data_i => slink_rx_dat_i,
          slink_rx_src_i => slink_rx_src_i,
          slink_rx_valid_i => slink_rx_val_i,
          slink_rx_last_i => slink_rx_lst_i,
          slink_rx_ready_o => slink_rx_rdy_o,
          slink_tx_data_o => slink_tx_dat_o,
          slink_tx_dst_o => slink_tx_dst_o,
          slink_tx_valid_o => slink_tx_val_o,
          slink_tx_last_o => slink_tx_lst_o,
          slink_tx_ready_i => slink_tx_rdy_i
        );
    END GENERATE;

    neorv32_slink_disabled :
    IF NOT IO_SLINK_EN GENERATE
      iodev_rsp(IODEV_SLINK) <= rsp_terminate_c;
      firq(FIRQ_SLINK_RX) <= '0';
      firq(FIRQ_SLINK_TX) <= '0';
      slink_rx_rdy_o <= '0';
      slink_tx_dat_o <= (OTHERS => '0');
      slink_tx_dst_o <= (OTHERS => '0');
      slink_tx_val_o <= '0';
      slink_tx_lst_o <= '0';
    END GENERATE;
    -- Cyclic Redundancy Check Unit (CRC) -----------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_crc_enabled :
    IF IO_CRC_EN GENERATE
      neorv32_crc_inst : ENTITY neorv32.neorv32_crc
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_CRC),
          bus_rsp_o => iodev_rsp(IODEV_CRC)
        );
    END GENERATE;

    neorv32_crc_disabled :
    IF NOT IO_CRC_EN GENERATE
      iodev_rsp(IODEV_CRC) <= rsp_terminate_c;
    END GENERATE;
    -- System Configuration Information Memory (SYSINFO) --------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sysinfo_enabled :
    IF io_sysinfo_en_c GENERATE
      neorv32_sysinfo_inst : ENTITY neorv32.neorv32_sysinfo
        GENERIC MAP(
          NUM_HARTS => num_cores_c,
          CLOCK_FREQUENCY => CLOCK_FREQUENCY,
          BOOT_MODE_SELECT => BOOT_MODE_SELECT,
          INT_BOOTLOADER_EN => bootrom_en_c,
          MEM_INT_IMEM_EN => MEM_INT_IMEM_EN,
          MEM_INT_IMEM_ROM => imem_as_rom_c,
          MEM_INT_IMEM_SIZE => imem_size_c,
          MEM_INT_DMEM_EN => MEM_INT_DMEM_EN,
          MEM_INT_DMEM_SIZE => dmem_size_c,
          ICACHE_EN => ICACHE_EN,
          ICACHE_NUM_BLOCKS => ICACHE_NUM_BLOCKS,
          ICACHE_BLOCK_SIZE => ICACHE_BLOCK_SIZE,
          DCACHE_EN => DCACHE_EN,
          DCACHE_NUM_BLOCKS => DCACHE_NUM_BLOCKS,
          DCACHE_BLOCK_SIZE => DCACHE_BLOCK_SIZE,
          XBUS_EN => XBUS_EN,
          XBUS_CACHE_EN => XBUS_CACHE_EN,
          XBUS_CACHE_NUM_BLOCKS => XBUS_CACHE_NUM_BLOCKS,
          XBUS_CACHE_BLOCK_SIZE => XBUS_CACHE_BLOCK_SIZE,
          OCD_EN => OCD_EN,
          OCD_AUTHENTICATION => OCD_AUTHENTICATION,
          IO_GPIO_EN => io_gpio_en_c,
          IO_CLINT_EN => IO_CLINT_EN,
          IO_UART0_EN => IO_UART0_EN,
          IO_UART1_EN => IO_UART1_EN,
          IO_SPI_EN => IO_SPI_EN,
          IO_SDI_EN => IO_SDI_EN,
          IO_TWI_EN => IO_TWI_EN,
          IO_TWD_EN => IO_TWD_EN,
          IO_PWM_EN => io_pwm_en_c,
          IO_WDT_EN => IO_WDT_EN,
          IO_TRNG_EN => IO_TRNG_EN,
          IO_CFS_EN => IO_CFS_EN,
          IO_NEOLED_EN => IO_NEOLED_EN,
          IO_GPTMR_EN => IO_GPTMR_EN,
          IO_ONEWIRE_EN => IO_ONEWIRE_EN,
          IO_DMA_EN => IO_DMA_EN,
          IO_SLINK_EN => IO_SLINK_EN,
          IO_CRC_EN => IO_CRC_EN
        )
        PORT MAP(
          clk_i => clk_i,
          rstn_i => rstn_sys,
          bus_req_i => iodev_req(IODEV_SYSINFO),
          bus_rsp_o => iodev_rsp(IODEV_SYSINFO)
        );
    END GENERATE;

    neorv32_sysinfo_disabled :
    IF NOT io_sysinfo_en_c GENERATE
      iodev_rsp(IODEV_SYSINFO) <= rsp_terminate_c;
    END GENERATE;
  END GENERATE; -- /io_system
  -- **************************************************************************************************************************
  -- On-Chip Debugger Complex
  -- **************************************************************************************************************************

  neorv32_ocd_enabled :
  IF OCD_EN GENERATE

    -- On-Chip Debugger - Debug Transport Module (DTM) ----------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dtm_inst : ENTITY neorv32.neorv32_debug_dtm
      GENERIC MAP(
        IDCODE_VERSION => (OTHERS => '0'), -- yet unused
        IDCODE_PARTID => (OTHERS => '0'), -- yet unused
        IDCODE_MANID => OCD_JEDEC_ID
      )
      PORT MAP(
        clk_i => clk_i,
        rstn_i => rstn_ext,
        jtag_tck_i => jtag_tck_i,
        jtag_tdi_i => jtag_tdi_i,
        jtag_tdo_o => jtag_tdo_o,
        jtag_tms_i => jtag_tms_i,
        dmi_req_o => dmi_req,
        dmi_rsp_i => dmi_rsp
      );

    -- On-Chip Debugger - Debug Module (DM) ---------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dm_inst : ENTITY neorv32.neorv32_debug_dm
      GENERIC MAP(
        NUM_HARTS => num_cores_c,
        AUTHENTICATOR => OCD_AUTHENTICATION
      )
      PORT MAP(
        clk_i => clk_i,
        rstn_i => rstn_ext,
        dmi_req_i => dmi_req,
        dmi_rsp_o => dmi_rsp,
        bus_req_i => iodev_req(IODEV_OCD),
        bus_rsp_o => iodev_rsp(IODEV_OCD),
        ndmrstn_o => dci_ndmrstn,
        halt_req_o => dci_haltreq
      );

  END GENERATE; -- /neorv32_ocd_enabled

  neorv32_debug_ocd_disabled :
  IF NOT OCD_EN GENERATE
    iodev_rsp(IODEV_OCD) <= rsp_terminate_c;
    jtag_tdo_o <= jtag_tdi_i; -- JTAG pass-through
    dci_ndmrstn <= '1';
    dci_haltreq <= (OTHERS => '0');
  END GENERATE;
END neorv32_top_rtl;