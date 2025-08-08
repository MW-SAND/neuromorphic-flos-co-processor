#include <neorv32.h>

volatile int32_t *image =   (int32_t *)0x80000400;
const int image_size = 784;
volatile int32_t *weights = (int32_t *)0x80001040;
const int weights_size = 10*784;
volatile int32_t *output =  (int32_t *)0x80008ac0;
const int output_size = 10;
volatile int32_t *xbus_out =   (int32_t *)0x80010000;

int main(int argc, char *argv[]) {
    // enable machine external interrupt
    neorv32_cpu_csr_set(CSR_MIE, (1 << CSR_MIE_MEIE));

    // enable sleep cntr in tb
    neorv32_gpio_pin_set(1, 1);
    neorv32_cpu_sleep();
    neorv32_gpio_pin_set(1, 0);

    // perform inference
    for (int i = 0; i < 10; i++) {
        output[i] = 0;
        for (int j = 0; j < 784; j++) {
            output[i] += image[j] * weights[j * 10 + i];
        }
        output[i] = output[i] >> 7;
    }

    for (int i = 0; i < 10; i++) {
        xbus_out[i] = output[i];
    }

    // enable sleep cntr in tb
    neorv32_gpio_pin_set(1, 1);
    neorv32_cpu_sleep();
    neorv32_gpio_pin_set(1, 0);

    // signal end of simulation
    neorv32_gpio_pin_set(0, 1);
}
