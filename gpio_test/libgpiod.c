#include <stdio.h>
#include <gpiod.h>
#include <unistd.h>

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s { gpiochip0 | gpiochip1 }  <GPIO_PIN_NAME>\nExample: %s gpiochip1 PAA.07", argv[0], argv[0]);
        return 1;
    }

    const char *gpioChipName = argv[1];
    const char *gpioPinName = argv[2];
    struct gpiod_chip *chip;
    struct gpiod_line *line;

    // Open the GPIO chip
    chip = gpiod_chip_open_by_name(gpioChipName);
    if (!chip) {
        perror("Failed to open GPIO chip");
        return 1;
    }

    // Request the GPIO line
    line = gpiod_chip_find_line(chip, gpioPinName);
    if (!line) {
        perror("Failed to get GPIO line");
        gpiod_chip_close(chip);
        return 1;
    }

    // Request the GPIO line as an output line
    if (gpiod_line_request_output(line, "example-gpio", 0) < 0) {
        perror("Failed to request GPIO line");
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }

    // Toggle the GPIO pin
    gpiod_line_set_value(line, 1);
    sleep(1); // Sleep for 1 second
    gpiod_line_set_value(line, 0);

    // Release the GPIO line and close the chip
    gpiod_line_release(line);
    gpiod_chip_close(chip);

    return 0;
}
