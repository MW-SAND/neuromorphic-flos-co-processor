#include <neorv32.h>

#define BAUD_RATE 19200

volatile int32_t *image =   (int32_t *)0x80000400;
const int image_size = 784;
volatile int32_t *weights = (int32_t *)0x80001040;
const int weights_size = 10*784;
volatile int32_t *output =  (int32_t *)0x80008ac0;
const int output_size = 10;

int main(int argc, char *argv[]) {
    // initialize NEORV32 run-time environment
    neorv32_rte_setup();

    // setup UART at default baud rate, no interrupts
    neorv32_uart0_setup(BAUD_RATE, 0);

    // show HPM hardware configuration
    uint32_t hpm_num = neorv32_cpu_hpm_get_num_counters();
    uint32_t hpm_width = neorv32_cpu_hpm_get_size();
    neorv32_uart0_printf("%u HPM counters detected, each %u bits wide\n", hpm_num, hpm_width);

    // stop all CPU counters including HPMs
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);

    // setup base counters if available
    if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR))) {
        neorv32_cpu_csr_write(CSR_MCYCLE,   0); neorv32_cpu_csr_write(CSR_MCYCLEH,   0);
        neorv32_cpu_csr_write(CSR_MINSTRET, 0); neorv32_cpu_csr_write(CSR_MINSTRETH, 0);
    }

    // clear HPM counters (low and high word);
    // there will be NO exception if we access a HPM counter register that has not been implemented
    // as long as Zihpm is implemented
    if (hpm_num > 0) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER3,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER3H,  0); }
    if (hpm_num > 1) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER4,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER4H,  0); }
    if (hpm_num > 2) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER5,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER5H,  0); }
    if (hpm_num > 3) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER6,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER6H,  0); }
    if (hpm_num > 4) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER7,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER7H,  0); }
    if (hpm_num > 5) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER8,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER8H,  0); }
    if (hpm_num > 6) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER9,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER9H,  0); }
    if (hpm_num > 7) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER10, 0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER10H, 0); }
    if (hpm_num > 8) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER11, 0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER11H, 0); }

    // configure events - one event per counter;
    // we can also configure more than one event; the HPM will increment if _any_ event triggers (logical OR);
    // there will be NO exception if we access a HPM event register that has not been implemented
    // as long as Zihpm is implemented
    if (hpm_num > 0) { neorv32_cpu_csr_write(CSR_MHPMEVENT3,  1 << HPMCNT_EVENT_COMPR);    } // executed compressed instruction
    if (hpm_num > 1) { neorv32_cpu_csr_write(CSR_MHPMEVENT4,  1 << HPMCNT_EVENT_WAIT_DIS); } // instruction dispatch wait cycle
    if (hpm_num > 2) { neorv32_cpu_csr_write(CSR_MHPMEVENT5,  1 << HPMCNT_EVENT_WAIT_ALU); } // multi-cycle ALU co-processor wait cycle
    if (hpm_num > 3) { neorv32_cpu_csr_write(CSR_MHPMEVENT6,  1 << HPMCNT_EVENT_BRANCH);   } // executed branch instruction
    if (hpm_num > 4) { neorv32_cpu_csr_write(CSR_MHPMEVENT7,  1 << HPMCNT_EVENT_BRANCHED); } // control flow transfer
    if (hpm_num > 5) { neorv32_cpu_csr_write(CSR_MHPMEVENT8,  1 << HPMCNT_EVENT_LOAD);     } // executed load operation
    if (hpm_num > 6) { neorv32_cpu_csr_write(CSR_MHPMEVENT9,  1 << HPMCNT_EVENT_STORE);    } // executed store operation
    if (hpm_num > 7) { neorv32_cpu_csr_write(CSR_MHPMEVENT10, 1 << HPMCNT_EVENT_WAIT_LSU); } // load-store unit memory wait cycle
    if (hpm_num > 8) { neorv32_cpu_csr_write(CSR_MHPMEVENT11, 1 << HPMCNT_EVENT_TRAP);     } // entered trap

    neorv32_uart0_puts("Starting inference...\n");

    // enable all CPU counters including HPMs
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0);

    // perform inference
    for (int i = 0; i < 10; i++) {
        output[i] = 0;
        for (int j = 0; j < 784; j++) {
            output[i] += image[j] * weights[j * 10 + i];
        }
        output[i] = output[i] >> 7;
    }

    // stop all CPU counters including HPMs
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);

    neorv32_uart0_puts("Finished inference:\n[");
    for (int i = 0; i < 9; i++) {
        neorv32_uart0_printf("0x%x, ", ((unsigned int*)output)[i]);
    }
    neorv32_uart0_printf("0x%x]\n", ((unsigned int*)output)[9]);

    // print HPM counter values (low word only)
    neorv32_uart0_printf("\nHPM results (low-words only):\n");
    if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR))) {
        neorv32_uart0_printf(" cycle (active clock cycles)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MCYCLE));
        neorv32_uart0_printf(" instret (retired instructions)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MINSTRET));
    }
    if (hpm_num > 0) { neorv32_uart0_printf(" HPM03 (compressed instructions)     : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER3));  }
    if (hpm_num > 1) { neorv32_uart0_printf(" HPM04 (instr. dispatch wait cycles) : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER4));  }
    if (hpm_num > 2) { neorv32_uart0_printf(" HPM05 (ALU wait cycles)             : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER5));  }
    if (hpm_num > 3) { neorv32_uart0_printf(" HPM06 (branch instructions)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER6));  }
    if (hpm_num > 4) { neorv32_uart0_printf(" HPM07 (control flow transfers)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER7));  }
    if (hpm_num > 5) { neorv32_uart0_printf(" HPM08 (load instructions)           : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER8));  }
    if (hpm_num > 6) { neorv32_uart0_printf(" HPM09 (store instructions)          : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER9));  }
    if (hpm_num > 7) { neorv32_uart0_printf(" HPM10 (load/store wait cycles)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER10)); }
    if (hpm_num > 8) { neorv32_uart0_printf(" HPM11 (entered traps)               : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER11)); }

    neorv32_gpio_pin_set(0, 1);
}
