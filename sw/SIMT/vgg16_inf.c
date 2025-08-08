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

#define NUM_BA_INCR 8
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

void FCLayerPE(uint32_t event_start_addr, int16_t *weight_addr_index, uint32_t weight_addr_rlc, int16_t *bias, bool has_bias, int16_t *output, uint32_t num_inputs, uint16_t num_neurons)
{
    uint16_t *event_addr = (uint16_t *)event_start_addr;

    // Disable all interrupts and write ps address
    NEORV32_CFS->REG[2] = (uint32_t)0x00000000;
    NEORV32_CFS->REG[0] = (uint32_t)(output - PE_COUNT) << BA_ADDR_OFFSET | PS_BA_IDX << BA_IDX_OFFSET | BA_OPC;

    int16_t task_buffer_capacity = TASK_BUFFER_SIZE;

    // Process multiple events
    for (uint32_t i = 0; i < num_inputs; i++)
    {
        // Extract the input value and address from the event
        uint16_t event = event_addr[i];
        int4_t input_val = ((event & 0x0000000F) ^ 0x08) - 0x08;
        uint16_t input_addr = event >> 4;

        // Calculate the weight address based on input address
        uint16_t *weights_start_addr = ((uint16_t *)weight_addr_index) + (input_addr * PE_COUNT);

        // Write input value, weight address and start task to accelerator task buffer
        NEORV32_CFS->REG[0] = (uint32_t)input_val << INPUT_VAL_VAL_OFFSET | INPUT_VAL_REG << INPUT_VAL_REG_OFFSET | INPUT_VAL_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)weights_start_addr << BA_ADDR_OFFSET | WEIGHTS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)0x0 << TASK_OFFSET | TASK_START_OPC;

        // Track task buffer capacity
        task_buffer_capacity -= 3;

        if (task_buffer_capacity <= 0)
        {
            // Enable all interrupts
            NEORV32_CFS->REG[2] = (uint32_t) 0x00000003;
            __asm volatile ("wfi");

            task_buffer_capacity = TASK_BUFFER_SIZE - 3;
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

void CONVLayerPE(uint32_t event_start_addr, int16_t *weight_addr_index, uint32_t weight_addr_rlc, int16_t *bias, bool has_bias, int16_t *output, uint32_t num_inputs, uint16_t *input_dim, uint16_t *kernel_dim, uint16_t *output_dim)
{
    uint32_t *event_addr = (uint32_t *)event_start_addr;

    // Enable empty interrupt
    NEORV32_CFS->REG[2] = (uint32_t)0x00000001;

    bool last_column_is_edge = false;


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
        uint16_t *weights_start_addr = ((uint16_t *)weight_addr_index) + (depth * PE_COUNT * kernel_dim[0]);

        // Use a padding of 2 for the column
        int32_t output_addr = row * (output_dim[1] + 2) * output_dim[2] + column * output_dim[2];

        if (task_buffer_capacity < 4)
        {
            __asm volatile("wfi");
            NEORV32_CFS->REG[2] = (uint32_t)0x00000001;
        }

        // Write input value, weight address, peratial sum address and start task to the accelerator
        NEORV32_CFS->REG[0] = (uint32_t)input_val << INPUT_VAL_VAL_OFFSET | INPUT_VAL_REG << INPUT_VAL_REG_OFFSET | INPUT_VAL_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)weights_start_addr << BA_ADDR_OFFSET | WEIGHTS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)(output + output_addr - PE_COUNT) << BA_ADDR_OFFSET | PS_BA_IDX << BA_IDX_OFFSET | BA_OPC;
        NEORV32_CFS->REG[0] = (uint32_t)0x0 << TASK_OFFSET | TASK_START_OPC;

        task_buffer_capacity -= 4;

        // Sleep until interrupt if the task buffer is full
        if (task_buffer_capacity < 4)
        {
            NEORV32_CFS->REG[2] = (uint32_t) 0x00000003;
        }
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

bool verifyOutputCONV(int16_t *output, int16_t *expected_output, uint16_t num_rows, uint16_t num_columns, uint16_t num_channels)
{
    // Loop over three dimensional outputs and skip padding
    int16_t *output_addr = output + (num_columns + 2) * num_channels + num_channels;

    for (uint16_t i = 0; i < num_rows; i++)
    {
        for (uint16_t j = 0; j < num_columns; j++)
        {
            for (uint16_t k = 0; k < num_channels; k++)
            {
                if (*output_addr != expected_output[(i * num_columns + j) * num_channels + k])
                    return false;

                output_addr += 1;
            }
        }

        output_addr += 2 * num_channels;
    }

    return true;
}

void printOutputCONV(int16_t *output, int16_t *expected_output, uint16_t num_rows, uint16_t num_columns, uint16_t num_channels)
{
    // Loop over three dimensional outputs and skip padding
    int16_t *output_addr = output + (num_columns + 2) * num_channels + num_channels;

    for (uint16_t i = 0; i < num_rows; i++)
    {
        for (uint16_t j = 0; j < num_columns; j++)
        {
            for (uint16_t k = 0; k < num_channels; k++)
            {
                if (*output_addr != expected_output[(i * num_columns + j) * num_channels + k])
                    neorv32_uart0_printf("%d: %d <--> %d\n", (i * num_columns + j) * num_channels + k, *output_addr, expected_output[(i * num_columns + j) * num_channels + k]);

                output_addr += 1;
            }
        }

        output_addr += 2 * num_channels;
    }
}

int main(int argc, char *argv[])
{
    // Read addresses from data_points.h 
    uint32_t event_start_addr = input;
    uint32_t weight_addr_rlc = weights_rlc;
    int16_t *weight_addr_index = (int16_t *)weights_index;
    int16_t *bias_addr = (int16_t *)bias;
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
    neorv32_rte_setup();

#if !SYNTHESIS
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
#endif

    if (!SYNTHESIS)
        configureUART();

    if (PE_COUNT > 0)
        configureCFS();

    // Enable HPM
    if (!SYNTHESIS)
        neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0);

    // Process the inputs
    if (LAYER_IS_FC)
    {
        if (PE_COUNT > 0)
            FCLayerPE(event_start_addr, weight_addr_index, weight_addr_rlc, bias_addr, has_bias, output_addr, num_inputs, num_neurons);
        else
            FCLayerRISCV(event_start_addr, weight_addr_index, weight_addr_rlc, bias_addr, has_bias, output_addr, num_inputs, num_neurons);
    }
    else
    {
        if (PE_COUNT > 0)
            CONVLayerPE(event_start_addr, weight_addr_index, weight_addr_rlc, bias_addr, has_bias, output_addr, num_inputs, input_dim, kernel_dim, output_dim);
        else
            CONVLayerRISCV(event_start_addr, weight_addr_index, weight_addr_rlc, bias_addr, has_bias, output_addr, num_inputs, input_dim, kernel_dim, output_dim);
    }

    // stop all CPU counters including HPMs
    if (!SYNTHESIS)
        neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);


    // Verify the output
    if (LAYER_IS_FC)
    {
        bool correct = verifyOutput(output_addr, expected_output_addr, num_neurons);

        if (!SYNTHESIS && !correct)
            printOutput(output_addr, expected_output_addr, num_neurons);
    }
    else
    {
        bool correct = verifyOutputCONV(output_addr, expected_output_addr, output_dim[0], output_dim[1], output_dim[2]);

        if (!SYNTHESIS && !correct)
            printOutputCONV(output_addr, expected_output_addr, output_dim[0], output_dim[1], output_dim[2]);
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
