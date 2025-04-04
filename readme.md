# Tako zmk config

The first thing you should do after assembling the keyboard is to calibrate the matrix.

You should be able to access ZMK logs through the console. See [ZMK docs](https://zmk.dev/docs/development/usb-logging) for details.

Next, you should adjust actuation and release values for each key in the `config/tako_drivers/kscan/kscan_gpio_ec.c` file. Basically, actuation should happen at the tactile bump, but you can prefer different levels depending on your typing style. If the key actuates too early, then increase the value in the `actuation_threshold`. If it's too late, then decrease it. After this is done, you should change the `release_threshold` to reflect the actuation change – reducing it by 50 should work great.

Don't map actuation levels too close to base reads (what you see in console when no touching keys).

//gpio_keys {
  //      compatible = "gpio-keys";
  //      wakeup_button: wakeup_button {
  //          label = "Wakeup Button";
            // - GPIO_ACTIVE_HIGH: Trigger on high signal
            // - GPIO_PULL_DOWN: Enable pull-down resistor
  //          gpios = <&gpio1 4 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
  //          wakeup-source;  // Marks this GPIO as a wakeup trigger
  //      };
  //  };