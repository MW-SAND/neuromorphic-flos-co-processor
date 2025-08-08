#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <neorv32.h>
#include <stdbool.h>
#include "datatype.h"
#include "acc_program.h"
#include "data_pointers.h"

#define PE_COUNT 8
#define SYNTHESIS false

#define LAYER_IS_FC false

#define TASK_BUFFER_SIZE 16

#define NUM_BA_INCR 16
#define BA_OFFSET 0

#define NUM_LC 3
#define LC_OFFSET (BA_OFFSET + NUM_BA_INCR)

#define NUM_TA 1
#define TA_OFFSET (LC_OFFSET + NUM_LC)

#define TI_OFFSET (TA_OFFSET + NUM_TA)

#define INPUT_VAL_REG 0
#define RELU_VAL_REG 3
#define RELU_VAL 0
#define SHIFT_VAL_REG 4

#define INPUT_VAL_REG_OFFSET 4
#define INPUT_VAL_OPC 2
#define INPUT_VAL_VAL_OFFSET 8

#define BA_OPC 1
#define PS_BA_IDX 0
#define WEIGHTS_BA_IDX 1
#define BITMAP_BA_IDX 2
#define BA_IDX_OFFSET 2
#define BA_ADDR_OFFSET 4

#define TASK_START_OPC 3
#define TASK_OFFSET 2

#define L0_START_OFFSET 4
#define L1_START_OFFSET 8
#define L2_START_OFFSET 12

int16_t task_buffer_capacity = TASK_BUFFER_SIZE;

void cfs_isr(void)
{
    // Clear interrupt
    NEORV32_CFS->REG[1] = (uint32_t)0x00000000;
    task_buffer_capacity = TASK_BUFFER_SIZE;
}

void enableHPM()
{
    // show HPM hardware configuration
    uint32_t hpm_num = neorv32_cpu_hpm_get_num_counters();

    // stop all CPU counters including HPMs
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);

    // setup base counters if available
    if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR)))
    {
        neorv32_cpu_csr_write(CSR_MCYCLE, 0);
        neorv32_cpu_csr_write(CSR_MCYCLEH, 0);
        neorv32_cpu_csr_write(CSR_MINSTRET, 0);
        neorv32_cpu_csr_write(CSR_MINSTRETH, 0);
    }

    // clear HPM counters (low and high word);
    if (hpm_num > 0)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER3, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER3H, 0);
    }
    if (hpm_num > 1)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER4, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER4H, 0);
    }
    if (hpm_num > 2)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER5, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER5H, 0);
    }
    if (hpm_num > 3)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER6, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER6H, 0);
    }
    if (hpm_num > 4)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER7, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER7H, 0);
    }
    if (hpm_num > 5)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER8, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER8H, 0);
    }
    if (hpm_num > 6)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER9, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER9H, 0);
    }
    if (hpm_num > 7)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER10, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER10H, 0);
    }
    if (hpm_num > 8)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER11, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER11H, 0);
    }
    if (hpm_num > 12)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER15, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER15H, 0);
    }

    // configure events - one event per counter;
    if (hpm_num > 0)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT3, 1 << HPMCNT_EVENT_COMPR);
    } // executed compressed instruction
    if (hpm_num > 1)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT4, 1 << HPMCNT_EVENT_WAIT_DIS);
    } // instruction dispatch wait cycle
    if (hpm_num > 2)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT5, 1 << HPMCNT_EVENT_WAIT_ALU);
    } // multi-cycle ALU co-processor wait cycle
    if (hpm_num > 3)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT6, 1 << HPMCNT_EVENT_BRANCH);
    } // executed branch instruction
    if (hpm_num > 4)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT7, 1 << HPMCNT_EVENT_BRANCHED);
    } // control flow transfer
    if (hpm_num > 5)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT8, 1 << HPMCNT_EVENT_LOAD);
    } // executed load operation
    if (hpm_num > 6)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT9, 1 << HPMCNT_EVENT_STORE);
    } // executed store operation
    if (hpm_num > 7)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT10, 1 << HPMCNT_EVENT_WAIT_LSU);
    } // load-store unit memory wait cycle
    if (hpm_num > 8)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT11, 1 << HPMCNT_EVENT_TRAP);
    } // entered trap
    if (hpm_num > 12)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT15, 1 << HPMCNT_EVENT_TRAP);
    } // entered trap
}

void printHPM()
{
    // Print the value of all HPM counters
    uint32_t hpm_num = neorv32_cpu_hpm_get_num_counters();

    neorv32_uart0_printf("\nHPM results (low-words only):\n");
    if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR)))
    {
        neorv32_uart0_printf(" cycle (active clock cycles)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MCYCLE));
        neorv32_uart0_printf(" instret (retired instructions)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MINSTRET));
    }
    if (hpm_num > 0)
    {
        neorv32_uart0_printf(" HPM03 (compressed instructions)     : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER3));
    }
    if (hpm_num > 1)
    {
        neorv32_uart0_printf(" HPM04 (instr. dispatch wait cycles) : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER4));
    }
    if (hpm_num > 2)
    {
        neorv32_uart0_printf(" HPM05 (ALU wait cycles)             : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER5));
    }
    if (hpm_num > 3)
    {
        neorv32_uart0_printf(" HPM06 (branch instructions)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER6));
    }
    if (hpm_num > 4)
    {
        neorv32_uart0_printf(" HPM07 (control flow transfers)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER7));
    }
    if (hpm_num > 5)
    {
        neorv32_uart0_printf(" HPM08 (load instructions)           : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER8));
    }
    if (hpm_num > 6)
    {
        neorv32_uart0_printf(" HPM09 (store instructions)          : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER9));
    }
    if (hpm_num > 7)
    {
        neorv32_uart0_printf(" HPM10 (load/store wait cycles)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER10));
    }
    if (hpm_num > 8)
    {
        neorv32_uart0_printf(" HPM11 (entered traps)               : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER11));
    }
    if (hpm_num > 12)
    {
        neorv32_uart0_printf(" HPM15 (total clock cyles)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER15));
    }
}

void configureCFS()
{
    // Configure Base Addresses
    for (int i = 0; i < NUM_BA_INCR; i++)
    {
        NEORV32_CFS->REG[64 + i * 2] = cfs_configuration[BA_OFFSET + i];
    }

    // Configure Loop Conditions
    for (int i = 0; i < NUM_LC; i++)
    {
        NEORV32_CFS->REG[56 + i] = cfs_configuration[LC_OFFSET + i];
    }

    // Configure Task Addresses
    for (int i = 0; i < NUM_TA; i++)
    {
        NEORV32_CFS->REG[52 + i] = cfs_configuration[TA_OFFSET + i];
    }

    // Configure Task Instructions
    for (int i = 0; i < (sizeof(cfs_configuration) / sizeof(cfs_configuration[0])) - TI_OFFSET; i++)
    {
        NEORV32_CFS->REG[128 + i * 2] = cfs_configuration[TI_OFFSET + i];
    }

    // Enable Global Interrupts
    neorv32_rte_handler_install(CFS_RTE_ID, cfs_isr);       // SPI to RTE
    neorv32_cpu_csr_set(CSR_MIE, 1 << CFS_FIRQ_ENABLE);     // enable SPI FIRQ
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode in

    return;
}

void configureUART()
{
    // show HPM hardware configuration
    uint32_t hpm_num = neorv32_cpu_hpm_get_num_counters();
    uint32_t hpm_width = neorv32_cpu_hpm_get_size();

    // setup UART at default baud rate, no interrupts
    neorv32_uart0_setup(19200, 0);
    neorv32_uart0_printf("%u HPM counters detected, each %u bits wide\n", hpm_num, hpm_width);
    neorv32_uart0_printf("Starting inference: VGG16 - %d PEs\n", PE_COUNT);

    return;
}

void FCLayerPE(uint32_t event_start_addr, uint32_t weight_addr_offset, int16_t *bitmap, int16_t *bias, bool has_bias, int16_t *output, uint32_t num_inputs, uint16_t num_neurons)
{
    uint8_t weight_size = 4;
    uint8_t weights_per_word = 16 / weight_size;

    int16_t task_buffer_capacity = TASK_BUFFER_SIZE;
    uint16_t *event_addr = (uint16_t *)event_start_addr;

    // Program -1 shift (SHIFTR) and output address
    NEORV32_CFS->REG[0] = (uint32_t)0xFFFF << INPUT_VAL_VAL_OFFSET | 0x6 << INPUT_VAL_REG_OFFSET | INPUT_VAL_OPC;
    NEORV32_CFS->REG[0] = (uint32_t)output << BA_ADDR_OFFSET | PS_BA_IDX << BA_IDX_OFFSET | BA_OPC;

    // Enable all interrupts
    NEORV32_CFS->REG[2] = (uint32_t)0x00000003;

    // Process multiple events
    for (uint32_t i = 0; i < num_inputs; i++)
    {
        // Extract the input value and address from the event
        uint16_t event = event_addr[i];
        int4_t input_val = ((event & 0x0000000F) ^ 0x08) - 0x08;
        uint16_t input_addr = event >> 4;

        // Calculate the weight and bitmap address based on input address
        uint16_t *weights_start_addr = ((uint16_t *)weight_addr_offset) + ((input_addr * num_neurons) / weights_per_word);
        uint16_t *bitmap_addr = ((uint16_t *)bitmap) + ((input_addr * num_neurons) / (weights_per_word * weights_per_word));

        // Write input value, weight address, bitmap address and start task to accelerator task buffer
        NEORV32_CFS->REG[0] = (uint32_t)input_val << INPUT_VAL_VAL_OFFSET | INPUT_VAL_REG << INPUT_VAL_REG_OFFSET | INPUT_VAL_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)weights_start_addr << BA_ADDR_OFFSET | WEIGHTS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)bitmap_addr << BA_ADDR_OFFSET | BITMAP_BA_IDX << BA_IDX_OFFSET | BA_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)0x0 << TASK_OFFSET | TASK_START_OPC;

        // Track task buffer capacity
        task_buffer_capacity -= 4;

        if (task_buffer_capacity < 4)
        {
            __asm volatile("wfi");

            task_buffer_capacity = TASK_BUFFER_SIZE - 4;
        }
    }

    // Write the second task for event generation
    NEORV32_CFS->REG[0] = (uint32_t)RELU_VAL << INPUT_VAL_VAL_OFFSET | RELU_VAL_REG << INPUT_VAL_REG_OFFSET | INPUT_VAL_OPC;
    NEORV32_CFS->REG[0] = (uint32_t)output << BA_ADDR_OFFSET | PS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
    if (has_bias)
        NEORV32_CFS->REG[0] = (uint32_t)bias << BA_ADDR_OFFSET | WEIGHTS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
    NEORV32_CFS->REG[2] = (uint32_t)0x00000001;
    NEORV32_CFS->REG[0] = (uint32_t)0x1 << TASK_OFFSET | TASK_START_OPC;

    __asm volatile("wfi");
    task_buffer_capacity = TASK_BUFFER_SIZE;

    return;
}

void CONVLayerPE(uint32_t event_start_addr, uint32_t weight_addr_offset, int16_t *bitmap, int16_t *bias, bool has_bias, int16_t *output, uint32_t num_inputs, uint16_t *input_dim, uint16_t *kernel_dim, uint16_t *output_dim)
{
    uint8_t weight_size = 4;
    uint8_t weights_per_word = 16 / weight_size;

    uint32_t *event_addr = (uint32_t *)event_start_addr;

    // Enable empty interrupt and write -1 to register
    NEORV32_CFS->REG[0] = (uint32_t)0xFFFF << INPUT_VAL_VAL_OFFSET | 0x6 << INPUT_VAL_REG_OFFSET | INPUT_VAL_OPC;
    NEORV32_CFS->REG[2] = (uint32_t)0x00000001;

    bool last_column_is_edge = false;
    bool first = true;

    for (uint32_t i = 0; i < num_inputs; i++)
    {
        // Extract the input value and address from the event
        uint32_t event = event_addr[i];
        int4_t input_val = ((event & 0x0000000F) ^ 0x08) - 0x08;
        uint32_t input_addr = event >> 4;

        // Calculate the 3-dimensional address in the input feature map based on a 1-dimensional address
        uint16_t row = input_addr / (input_dim[2] * input_dim[1]);
        uint16_t column = (input_addr / input_dim[2]) % input_dim[1];
        uint16_t depth = input_addr % input_dim[2];

        // Calculate the weight address offset based on the input depth and kernel dimensions
        uint32_t weight_addr = depth * kernel_dim[0] * kernel_dim[1] * kernel_dim[2];
        int32_t output_addr = (row - 1) * output_dim[1] * output_dim[2] + (column - 1) * output_dim[2];

        // Loop Counter offsets 
        uint8_t L0_start = 0;
        uint8_t L1_start = 0;

        // Check whether the input even tis at an edge row
        if (row == 0)
        {
            // Offset the weight and partial sum address to only multiply the input event with the second and third row of the kernel
            L0_start = 1;
            weight_addr += kernel_dim[1] * kernel_dim[2];
            output_addr += output_dim[1] * output_dim[2];
        }
        else if (row == output_dim[0] - 1)
        {
            // Prevent multiplication with the third row of the kernel
            L0_start = 1;
        }

        // Check whether the input event is at an edge column
        if (column == 0)
        {
            // Offset the weight and partial sum address to only multiply the input event with the second and third column of the kernel
            L1_start = 1;
            weight_addr += kernel_dim[2];
            output_addr += output_dim[2];

            if (last_column_is_edge == false)
            {
                // Reconfigure the accelerator
                if (first == false)
                {
                    __asm volatile("wfi");
                }
                else
                    first = false;

                // Set base address increments to jump over first column for second and third row of the weight kernel
                NEORV32_CFS->REG[64] = 64 << 16 | (cfs_configuration[BA_OFFSET] + 256);
                NEORV32_CFS->REG[66] = 16;
                last_column_is_edge = true;
            }
        }
        else if (column == output_dim[1] - 1)
        {
            // Prevent multiplication with the third column of the kernel
            L1_start = 1;
            if (last_column_is_edge == false)
            {
                // Reconfigure the accelerator
                if (first == false)
                {
                    __asm volatile("wfi");
                }
                else
                    first = false;

                // Set base address increments to jump over third column for first and second row of the weight kernel 
                NEORV32_CFS->REG[64] = 64 << 16 | (cfs_configuration[BA_OFFSET] + 256);
                NEORV32_CFS->REG[66] = 16;
                last_column_is_edge = true;
            }
        }
        else if (last_column_is_edge == true)
        {
            // Reconfigure the accelerator
            if (first == false)
            {
                __asm volatile("wfi");
            }
            else
                first = false;

            // Remove previous set offsets to jump over columns
            NEORV32_CFS->REG[64] = 0 << 16 | cfs_configuration[BA_OFFSET];
            NEORV32_CFS->REG[66] = 0;
            last_column_is_edge = false;
        }

        // Sleep until interrupt if the task buffer is full
        if (task_buffer_capacity < 5)
        {
            if (last_column_is_edge == false)
            {
                NEORV32_CFS->REG[2] = (uint32_t)0x00000002;
            }
            else
            {
                NEORV32_CFS->REG[2] = (uint32_t)0x00000003;
            }
            __asm volatile("wfi");
            NEORV32_CFS->REG[2] = (uint32_t)0x00000001;
        }

        // Calculate the address of the weights and bitmaps in memory
        uint16_t *weights_start_addr = ((uint16_t *)weight_addr_offset) + (weight_addr / weights_per_word);
        uint16_t *bitmap_addr = ((uint16_t *)bitmap) + (weight_addr / (weights_per_word * weights_per_word));

        // Write input value, weight address, bitmap address, peratial sum address and start task to the accelerator
        NEORV32_CFS->REG[0] = (uint32_t)input_val << INPUT_VAL_VAL_OFFSET | INPUT_VAL_REG << INPUT_VAL_REG_OFFSET | INPUT_VAL_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)weights_start_addr << BA_ADDR_OFFSET | WEIGHTS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)(output + output_addr) << BA_ADDR_OFFSET | PS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)bitmap_addr << BA_ADDR_OFFSET | BITMAP_BA_IDX << BA_IDX_OFFSET | BA_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)(L1_start & 0x0F) << L1_START_OFFSET | (L0_start & 0x0F) << L0_START_OFFSET | 0x0 << TASK_OFFSET | TASK_START_OPC;

        task_buffer_capacity -= 5;
    }

    // Write the second task for event generation
    NEORV32_CFS->REG[0] = (uint32_t)RELU_VAL << INPUT_VAL_VAL_OFFSET | RELU_VAL_REG << INPUT_VAL_REG_OFFSET | INPUT_VAL_OPC;
    NEORV32_CFS->REG[0] = (uint32_t)output << BA_ADDR_OFFSET | PS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
    if (has_bias)
        NEORV32_CFS->REG[0] = (uint32_t)bias << BA_ADDR_OFFSET | WEIGHTS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
    NEORV32_CFS->REG[2] = (uint32_t)0x00000001;
    NEORV32_CFS->REG[0] = (uint32_t)0x1 << TASK_OFFSET | TASK_START_OPC;

    // Wait until task completion
    __asm volatile("wfi");
    task_buffer_capacity = TASK_BUFFER_SIZE;

    return;
}

void FCLayerRISCV(uint32_t event_start_addr, uint32_t weight_addr_offset, int16_t *bias, bool has_bias, int16_t *output, uint32_t num_inputs, uint16_t num_neurons)
{
    uint8_t weight_size = 4;
    uint8_t weights_per_word = 16 / weight_size;
    uint16_t weight_mask = 0;
    uint16_t weight_sign = (1 << (weight_size - 1));

    uint16_t *event_addr = (uint16_t *)event_start_addr;

    // Generate weight mask
    for (uint8_t i = 0; i < weight_size; i++)
        weight_mask |= (1 << i);

    // Process multiple inputs
    for (uint16_t i = 0; i < num_inputs; i++)
    {
        // Extract input value and address
        uint16_t event = event_addr[i];
        int4_t input_val = ((event & 0x0000000F) ^ 0x08) - 0x08;
        uint16_t input_addr = event >> 4;

        // Calculate weight address
        uint16_t *weights_start_addr = ((uint16_t *)weight_addr_offset) + ((input_addr * num_neurons) / weights_per_word);

        // Loop over nuerons
        for (uint16_t j = 0; j < num_neurons; j += weight_size)
        {
            uint16_t weights = weights_start_addr[j >> 2];

            // Extract four weights and update the neurons
            for (uint16_t k = 0; k < weights_per_word; k++)
            {
                int4_t weight = ((weights & weight_mask) ^ weight_sign) - weight_sign;
                weights = weights >> weight_size;

                output[j + k] += weight * input_val;
            }
        }
    }

    // Perform bias addition and event generation
    for (uint16_t i = 0; i < num_neurons; i++)
    {
        if (has_bias)
            output[i] += bias[i];

        if (output[i] < 0)
            output[i] = 0;
    }

    return;
}

void CONVLayerRISCV(uint32_t event_start_addr, uint32_t weight_addr_offset, int16_t *bias, bool has_bias, int16_t *output, uint32_t num_inputs, uint16_t *input_dim, uint16_t *kernel_dim, uint16_t *output_dim)
{
    uint8_t weight_size = 4;
    uint8_t weights_per_word = 16 / weight_size;
    uint16_t weight_mask = 0;
    uint16_t weight_sign = (1 << (weight_size - 1));

    uint32_t *event_addr = (uint32_t *)event_start_addr;

    // Generate weight mask
    for (uint8_t i = 0; i < weight_size; i++)
        weight_mask |= (1 << i);

    // Process multiple inputs
    for (uint16_t i = 0; i < num_inputs; i++)
    {
        // Extract input value and address
        uint32_t event = event_addr[i];
        int4_t input_val = ((event & 0x0000000F) ^ 0x08) - 0x08;
        uint32_t input_addr = event >> 4;

        // Calculate the 3D convolutional addresses
        uint16_t row = input_addr / (input_dim[2] * input_dim[1]);
        uint16_t column = (input_addr / input_dim[2]) % input_dim[1];
        uint16_t depth = input_addr % input_dim[2];

        // Calculate the weight address in memory
        uint32_t weight_addr = depth * kernel_dim[0] * kernel_dim[1] * kernel_dim[2];
        uint16_t *weights_start_addr = ((uint16_t *)weight_addr_offset) + (weight_addr / weights_per_word);

        // Calculate start address of the partial sums
        int32_t output_addr = (row - 1) * output_dim[1] * output_dim[2] + (column - 1) * output_dim[2];

        uint32_t cur_weight_idx = 0;
        int32_t cur_out_idx = 0;
        // Loop over first kernel dimension
        for (int16_t r = -kernel_dim[0] / 2; r <= kernel_dim[0] / 2; r++)
        {
            // Check boundaries of the output feature map
            if (row + r >= 0 && row + r < output_dim[0])
            {
                // Loop over second kernel dimension
                for (int16_t co = -kernel_dim[1] / 2; co <= kernel_dim[1] / 2; co++)
                {
                    // Check boundaries of hte output feature map
                    if (column + co >= 0 && column + co < output_dim[1])
                    {
                        // Loop over third kernel dimension
                        for (uint16_t ca = 0; ca < kernel_dim[2]; ca += weight_size)
                        {
                            // Extract 4 weights and perform the neuron update
                            uint16_t weights = weights_start_addr[cur_weight_idx >> 2];
                            for (uint16_t j = 0; j < weights_per_word; j++)
                            {
                                int4_t weight = ((weights & weight_mask) ^ weight_sign) - weight_sign;
                                weights = weights >> weight_size;

                                output[output_addr + cur_out_idx] += weight * input_val;
                                cur_weight_idx += 1;
                                cur_out_idx += 1;
                            }
                        }
                    }
                    else
                    {
                        // Increment the indexes
                        cur_weight_idx += kernel_dim[2];
                        cur_out_idx += output_dim[2];
                    }
                }

                // Increment the partial sum index
                cur_out_idx += (output_dim[1] - kernel_dim[1]) * output_dim[2];
            }
            else
            {
                // Increment the indexes
                cur_out_idx += output_dim[1] * output_dim[2];
                cur_weight_idx += kernel_dim[1] * kernel_dim[2];
            }
        }
    }

    // Event generation, apply bias and relu activation function
    for (uint16_t i = 0; i < output_dim[0] * output_dim[1] * output_dim[2]; i += output_dim[2])
    {
        for (uint16_t j = 0; j < output_dim[2]; j++)
        {
            if (has_bias)
                output[i + j] += bias[j];

            if (output[i + j] < 0)
                output[i + j] = 0;
        }
    }
    return;
}

bool verifyOutput(int16_t *output, int16_t *expected_output, uint16_t num_outputs)
{
    // Loop over outputs and check against expected output
    for (uint16_t i = 0; i < num_outputs; i++)
    {
        if (output[i] != expected_output[i])
            return false;
    }

    return true;
}

void printOutput(int16_t *output, int16_t *expected_output, uint16_t num_outputs)
{
    // Print all outputs and expected outputs
    for (uint16_t i = 0; i < num_outputs; i++)
    {
        neorv32_uart0_printf("%d: %d <--> %d\n", i, output[i], expected_output[i]);
    }
}

int main(int argc, char *argv[])
{
    // Read addresses from data_points.h 
    uint32_t event_start_addr = input;
    uint32_t weight_addr_offset = weights;
    int16_t *bias_addr = (int16_t *)bias;
    int16_t *bitmap_addr = (int16_t *)bitmap;
    int16_t *output_addr = (int16_t *)unallocated;
    int16_t *expected_output_addr = (int16_t *)output;

    bool has_bias = true;
    uint32_t num_inputs = 9693;
    uint16_t num_neurons = 512;
    uint16_t input_dim[3] = {16, 16, 64};
    uint16_t kernel_dim[3] = {3, 3, 128};
    uint16_t output_dim[3] = {16, 16, 128};

// initialize NEORV32 run-time environment
// Enable HPM
#if !SYNTHESIS
    neorv32_rte_setup();

    // show HPM hardware configuration
    uint32_t hpm_num = neorv32_cpu_hpm_get_num_counters();

    // stop all CPU counters including HPMs
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);

    // setup base counters if available
    if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR)))
    {
        neorv32_cpu_csr_write(CSR_MCYCLE, 0);
        neorv32_cpu_csr_write(CSR_MCYCLEH, 0);
        neorv32_cpu_csr_write(CSR_MINSTRET, 0);
        neorv32_cpu_csr_write(CSR_MINSTRETH, 0);
    }

    // clear HPM counters (low and high word);
    if (hpm_num > 0)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER3, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER3H, 0);
    }
    if (hpm_num > 1)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER4, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER4H, 0);
    }
    if (hpm_num > 2)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER5, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER5H, 0);
    }
    if (hpm_num > 3)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER6, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER6H, 0);
    }
    if (hpm_num > 4)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER7, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER7H, 0);
    }
    if (hpm_num > 5)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER8, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER8H, 0);
    }
    if (hpm_num > 6)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER9, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER9H, 0);
    }
    if (hpm_num > 7)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER10, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER10H, 0);
    }
    if (hpm_num > 8)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER11, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER11H, 0);
    }
    if (hpm_num > 12)
    {
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER15, 0);
        neorv32_cpu_csr_write(CSR_MHPMCOUNTER15H, 0);
    }

    // configure events - one event per counter;
    if (hpm_num > 0)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT3, 1 << HPMCNT_EVENT_COMPR);
    } // executed compressed instruction
    if (hpm_num > 1)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT4, 1 << HPMCNT_EVENT_WAIT_DIS);
    } // instruction dispatch wait cycle
    if (hpm_num > 2)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT5, 1 << HPMCNT_EVENT_WAIT_ALU);
    } // multi-cycle ALU co-processor wait cycle
    if (hpm_num > 3)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT6, 1 << HPMCNT_EVENT_BRANCH);
    } // executed branch instruction
    if (hpm_num > 4)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT7, 1 << HPMCNT_EVENT_BRANCHED);
    } // control flow transfer
    if (hpm_num > 5)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT8, 1 << HPMCNT_EVENT_LOAD);
    } // executed load operation
    if (hpm_num > 6)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT9, 1 << HPMCNT_EVENT_STORE);
    } // executed store operation
    if (hpm_num > 7)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT10, 1 << HPMCNT_EVENT_WAIT_LSU);
    } // load-store unit memory wait cycle
    if (hpm_num > 8)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT11, 1 << HPMCNT_EVENT_TRAP);
    } // entered trap
    if (hpm_num > 12)
    {
        neorv32_cpu_csr_write(CSR_MHPMEVENT15, 1 << HPMCNT_EVENT_TRAP);
    } // entered trap
#else
    neorv32_rte_setup();
#endif

#if !SYNTHESIS
    configureUART();
#endif

    if (PE_COUNT > 0)
        configureCFS();

// Enable HPM
#if !SYNTHESIS
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0);
#endif

    // Process the inputs
    if (LAYER_IS_FC)
    {
        if (PE_COUNT > 0)
            FCLayerPE(event_start_addr, weight_addr_offset, bitmap_addr, bias_addr, has_bias, output_addr, num_inputs, num_neurons);
        else
            FCLayerRISCV(event_start_addr, weight_addr_offset, bias_addr, has_bias, output_addr, num_inputs, num_neurons);
    }
    else
    {
        if (PE_COUNT > 0)
            CONVLayerPE(event_start_addr, weight_addr_offset, bitmap_addr, bias_addr, has_bias, output_addr, num_inputs, input_dim, kernel_dim, output_dim);
        else
            CONVLayerRISCV(event_start_addr, weight_addr_offset, bias_addr, has_bias, output_addr, num_inputs, input_dim, kernel_dim, output_dim);
    }

// stop all CPU counters including HPMs
#if !SYNTHESIS
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);
#endif

    // Verify the output
    if (LAYER_IS_FC)
    {
        bool correct = verifyOutput(output_addr, expected_output_addr, num_neurons);

        if (!SYNTHESIS && !correct)
            printOutput(output_addr, expected_output_addr, num_neurons);
    }
    else
    {
        uint32_t num_outputs = output_dim[0] * output_dim[1] * output_dim[2];
        bool correct = verifyOutput(output_addr, expected_output_addr, num_outputs);

        if (!SYNTHESIS && !correct)
            printOutput(output_addr, expected_output_addr, num_outputs);
    }

// Print HPM
#if !SYNTHESIS

    neorv32_uart0_printf("\nHPM results (low-words only):\n");
    if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR)))
    {
        neorv32_uart0_printf(" cycle (active clock cycles)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MCYCLE));
        neorv32_uart0_printf(" instret (retired instructions)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MINSTRET));
    }
    if (hpm_num > 0)
    {
        neorv32_uart0_printf(" HPM03 (compressed instructions)     : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER3));
    }
    if (hpm_num > 1)
    {
        neorv32_uart0_printf(" HPM04 (instr. dispatch wait cycles) : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER4));
    }
    if (hpm_num > 2)
    {
        neorv32_uart0_printf(" HPM05 (ALU wait cycles)             : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER5));
    }
    if (hpm_num > 3)
    {
        neorv32_uart0_printf(" HPM06 (branch instructions)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER6));
    }
    if (hpm_num > 4)
    {
        neorv32_uart0_printf(" HPM07 (control flow transfers)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER7));
    }
    if (hpm_num > 5)
    {
        neorv32_uart0_printf(" HPM08 (load instructions)           : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER8));
    }
    if (hpm_num > 6)
    {
        neorv32_uart0_printf(" HPM09 (store instructions)          : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER9));
    }
    if (hpm_num > 7)
    {
        neorv32_uart0_printf(" HPM10 (load/store wait cycles)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER10));
    }
    if (hpm_num > 8)
    {
        neorv32_uart0_printf(" HPM11 (entered traps)               : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER11));
    }
    if (hpm_num > 12)
    {
        neorv32_uart0_printf(" HPM15 (total clock cyles)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER15));
    }
#endif
}
