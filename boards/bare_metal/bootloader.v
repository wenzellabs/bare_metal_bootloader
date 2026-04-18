// bare_metal USB bootloader - bootloader.v
// based on TinyFPGA BX bootloader
// iCE40UP5K-SG48ITR, 12 MHz crystal
//
// Mechanism:
//   Slot 0 = this bootloader
//   Slot 1 = user design
//   Slot 2 = user design
//   Slot 3 = user design
//   SB_WARMBOOT S1=0 S0=1 -> boots to slot 1
//
// btn_ok bypass:
//   If btn_ok (active low) is held at power-on / reset,
//   we skip USB entirely and warmboot straight to the user image.
//
// btn_down stay:
//   If btn_down (active low) is held at power-on / reset,
//   the auto-boot timer is disabled - bootloader stays up indefinitely,
//   useful for programming without a USB host present at power-on.
//
// Otherwise we run the TinyFPGA bootloader (USB CDC-ACM + SPI bridge).
// If no USB host sends SOF within ~16 s, we auto-warmboot to user image.
//
// USB pull-up: 1k5 on D+ controlled by USB_DET (pin 37, active high).
//   Drive high to enumerate on the bus, low to disconnect.

module bootloader (
  input  pin_clk_12M,       // 12 MHz crystal on dedicated clock pin 35

  inout  pin_usbp,          // USB D+ (pin 42, IOT_51a)
  inout  pin_usbn,          // USB D- (pin 38, IOT_50b)
  output pin_usb_det,       // USB_DET (pin 37, IOT_36b) - high enables 1k5 pull-up on D+

  input  pin_btn_ok,        // btn_ok, active low with external pull-up
  input  pin_btn_down,      // btn_down, active low - hold to stay in bootloader

  output pin_led_index,     // white LED index  finger (pin 39)
  output pin_led_middle,    // white LED middle finger (pin 40)
  output pin_led_pinky,     // white LED pinky  finger (pin 41)

  input  pin_spi_miso,      // SPI flash MISO (dedicated pin 17)
  output pin_spi_cs,        // SPI flash CS   (dedicated pin 16)
  output pin_spi_mosi,      // SPI flash MOSI (dedicated pin 14)
  output pin_spi_sck,       // SPI flash SCK  (dedicated pin 15)

  output pin_spi_wp,        // SPI flash /WP   (pin 18, IOB_31b) - drive high to disable write-protect
  output pin_spi_hold       // SPI flash /HOLD (pin 19, IOB_29b) - drive high to disable hold
);

  // ============================================================================
  // Dedicated SPI pins (14-17) on UP5K
  // ============================================================================
  // The iCE40 UP5K has an SB_SPI hard block on these pads, but if we do NOT
  // instantiate it, the pads are available as regular GPIO - same approach as
  // the Fomu (also UP5K) bootloader.  Instantiating SB_SPI (even "disabled")
  // connects its output enables to the pads and creates bus contention.

  // ============================================================================
  // PLL: 12 MHz -> 48 MHz
  // ============================================================================
  // Pin 35 is a dedicated clock pad, so we use SB_PLL40_PAD (not _CORE).
  // Parameters from: icepll -i 12 -o 48
  wire clk_48mhz;
  wire lock;
  wire reset = !lock;

  SB_PLL40_PAD #(
    .DIVR      (4'b0000),     // DIVR = 0
    .DIVF      (7'b0111111),  // DIVF = 63
    .DIVQ      (3'b100),      // DIVQ = 4
    .FILTER_RANGE(3'b001),
    .FEEDBACK_PATH("SIMPLE"),
    .DELAY_ADJUSTMENT_MODE_FEEDBACK("FIXED"),
    .FDA_FEEDBACK(4'b0000),
    .DELAY_ADJUSTMENT_MODE_RELATIVE("FIXED"),
    .FDA_RELATIVE(4'b0000),
    .SHIFTREG_DIV_MODE(2'b00),
    .PLLOUT_SELECT("GENCLK"),
    .ENABLE_ICEGATE(1'b0)
  ) usb_pll_inst (
    .PACKAGEPIN    (pin_clk_12M),
    .PLLOUTCORE    (clk_48mhz),
    .PLLOUTGLOBAL  (),
    .EXTFEEDBACK   (),
    .DYNAMICDELAY  (),
    .RESETB        (1'b1),
    .BYPASS        (1'b0),
    .LATCHINPUTVALUE(),
    .LOCK          (lock),
    .SDI           (),
    .SDO           (),
    .SCLK          ()
  );

  // ============================================================================
  // Flash /WP and /HOLD - drive high to keep flash fully operational
  // ============================================================================
  // After FPGA configuration, these GPIO pins float. If /HOLD goes low,
  // the flash freezes its SPI output - causing all-zero reads.
  assign pin_spi_wp   = 1'b1;
  assign pin_spi_hold = 1'b1;

  // ============================================================================
  // Clock divider: 48 -> 24 -> 12 MHz
  // ============================================================================
  // The TinyFPGA bootloader expects both clk_48mhz and a slower 'clk'.
  // The host_presence_timer counts at 'clk' rate; timeout is 196 000 000
  // which gives ~16.3 s at 12 MHz.
  reg clk_24mhz = 0;
  reg clk_12mhz = 0;
  always @(posedge clk_48mhz) clk_24mhz <= !clk_24mhz;
  always @(posedge clk_24mhz) clk_12mhz <= !clk_12mhz;

  wire clk = clk_12mhz;

  // ============================================================================
  // btn_ok bypass - skip bootloader, go straight to user image
  // ============================================================================
  // Sample btn_ok once after PLL lock. If held low -> warmboot to user design.
  // No debounce needed: button is either held during power-on/reset or not,
  // and PLL lock time (~100 µs) already provides a stable sampling point.
  reg bypass_sampled = 0;
  reg bypass_trigger = 0;
  reg stay_in_bootloader = 0;    // btn_down held at boot -> inhibit autoboot

  always @(posedge clk) begin
    if (reset) begin
      bypass_sampled     <= 0;
      bypass_trigger     <= 0;
      stay_in_bootloader <= 0;
    end else if (!bypass_sampled) begin
      bypass_sampled     <= 1;
      bypass_trigger     <= !pin_btn_ok;    // active low: pressed = go to user
      stay_in_bootloader <= !pin_btn_down;  // active low: pressed = stay in BL
    end
  end

  // ============================================================================
  // Auto-boot timer - warmboot to user if no host / tinyprog activity
  // ============================================================================
  // After ~1 s with no USB host or SPI activity, warmboot to user image.
  // Resets when:
  //   - SPI CS goes low (tinyprog doing a transaction), OR
  //   - USB TX fires (host is enumerating / talking to us)
  // Once any activity is seen the timer is permanently disarmed.
  reg  spi_cs_prev = 1;
  reg  usb_tx_prev = 0;
  reg  host_activity = 0;         // latches on any USB or SPI activity
  reg [23:0] autoboot_cnt = 0;    // 2^23 / 12 MHz ≈ 0.7 s - close to 1 s
  reg  autoboot_trigger = 0;

  always @(posedge clk) begin
    if (reset) begin
      spi_cs_prev      <= 1;
      usb_tx_prev      <= 0;
      host_activity    <= 0;
      autoboot_cnt     <= 0;
      autoboot_trigger <= 0;
    end else begin
      spi_cs_prev <= pin_spi_cs;
      usb_tx_prev <= usb_tx_en;

      // Detect SPI CS falling edge or USB TX rising edge
      if ((spi_cs_prev && !pin_spi_cs) || (!usb_tx_prev && usb_tx_en))
        host_activity <= 1;

      // Count up if no activity has ever been seen and not held in BL
      if (!host_activity && !autoboot_trigger && !stay_in_bootloader) begin
        if (autoboot_cnt[23])
          autoboot_trigger <= 1;
        else
          autoboot_cnt <= autoboot_cnt + 1'b1;
      end
    end
  end

  // ============================================================================
  // 3-LED breathing with 120° phase shift
  // ============================================================================
  // Triangle wave: 8-bit PWM value ramps 0->255->0 over ~512 µs steps.
  // We use a 10-bit microsecond counter (from 12 MHz clk) and a 9-bit
  // phase accumulator.  Three LEDs are offset by 170 (≈ 512/3 ≈ 120°).
  reg [5:0] breath_ns = 0;       // divide clk by ~48 -> 1 µs ticks (shared with BL core idea)
  wire breath_ns_rst = (breath_ns == 47);
  always @(posedge clk) breath_ns <= breath_ns_rst ? 0 : breath_ns + 1'b1;

  reg [9:0] breath_us = 0;
  wire breath_us_rst = (breath_us == 999);
  always @(posedge clk) if (breath_ns_rst) breath_us <= breath_us_rst ? 0 : breath_us + 1'b1;

  reg [8:0] breath_phase = 0;    // 0..511 triangle base
  always @(posedge clk) if (breath_ns_rst && breath_us_rst) breath_phase <= breath_phase + 1'b1;

  // Triangle function: phase 0..255 -> ramp up, 256..511 -> ramp down
  function [7:0] triangle;
    input [8:0] ph;
    triangle = ph[8] ? ~ph[7:0] : ph[7:0];
  endfunction

  wire [7:0] pwm_index  = triangle(breath_phase);
  wire [7:0] pwm_middle = triangle(breath_phase + 9'd170);
  wire [7:0] pwm_pinky  = triangle(breath_phase + 9'd341);

  reg [7:0] pwm_cnt = 0;
  always @(posedge clk) pwm_cnt <= pwm_cnt + 1'b1;

  assign pin_led_index  = pwm_index  > pwm_cnt;
  assign pin_led_middle = pwm_middle > pwm_cnt;
  assign pin_led_pinky  = pwm_pinky  > pwm_cnt;

  // ============================================================================
  // USB_DET - control the 1k5 pull-up on D+
  // ============================================================================
  // Only assert (enumerate) after the bypass window has closed without
  // btn_ok being held - i.e. we committed to running the bootloader.
  // While bypass_armed is still 1 we keep USB_DET low so we don't glitch
  // onto the bus if we're about to warmboot straight to the user image.
  // Also deassert when the bootloader core requests boot (tinyprog "boot"
  // command or host timeout) so the host sees a clean disconnect.
  assign pin_usb_det = bypass_sampled && !bypass_trigger && !autoboot_trigger && !boot_from_bootloader;

  // ============================================================================
  // SB_WARMBOOT - interface to multiboot
  // ============================================================================
  // S1=0, S0=1 -> selects image slot 1 (user design)
  // BOOT pulse triggers the reconfig
  wire boot_from_bootloader;     // driven by tinyfpga_bootloader (timeout or USB cmd)

  // When stay_in_bootloader is set, ignore the core's internal timeout.
  // The core also fires boot_from_bootloader on a USB "boot" command from
  // tinyprog - we still want that, but can't distinguish here. Acceptable
  // trade-off: with btn_down held, tinyprog "boot" command also won't work.
  // In practice, tinyprog always programs then boots, so if btn_down is held
  // the user explicitly wants to stay.
  wire boot = (!stay_in_bootloader && boot_from_bootloader) || bypass_trigger || autoboot_trigger;

  SB_WARMBOOT warmboot_inst (
    .S1   (1'b0),
    .S0   (1'b1),
    .BOOT (boot)
  );

  // ============================================================================
  // USB tristate buffers - SB_IO primitives
  // ============================================================================
  wire usb_p_tx;
  wire usb_n_tx;
  wire usb_p_rx;
  wire usb_n_rx;
  wire usb_p_rx_io;
  wire usb_n_rx_io;
  wire usb_tx_en;

  // When transmitting, feed back the idle state to RX (J state: D+=1, D-=0)
  assign usb_p_rx = usb_tx_en ? 1'b1 : usb_p_rx_io;
  assign usb_n_rx = usb_tx_en ? 1'b0 : usb_n_rx_io;

  SB_IO #(
    .PIN_TYPE(6'b1010_01)  // tristatable output
  ) usbp_buf (
    .PACKAGE_PIN   (pin_usbp),
    .OUTPUT_ENABLE (usb_tx_en),
    .D_IN_0        (usb_p_rx_io),
    .D_OUT_0       (usb_p_tx)
  );

  SB_IO #(
    .PIN_TYPE(6'b1010_01)  // tristatable output
  ) usbn_buf (
    .PACKAGE_PIN   (pin_usbn),
    .OUTPUT_ENABLE (usb_tx_en),
    .D_IN_0        (usb_n_rx_io),
    .D_OUT_0       (usb_n_tx)
  );

  // ============================================================================
  // TinyFPGA bootloader core
  // ============================================================================
  tinyfpga_bootloader tinyfpga_bootloader_inst (
    .clk_48mhz (clk_48mhz),
    .clk       (clk),
    .reset     (reset),

    .usb_p_tx  (usb_p_tx),
    .usb_n_tx  (usb_n_tx),
    .usb_p_rx  (usb_p_rx),
    .usb_n_rx  (usb_n_rx),
    .usb_tx_en (usb_tx_en),

    .led       (),              // LED now driven by our 3-LED breather

    .spi_miso  (pin_spi_miso),
    .spi_cs    (pin_spi_cs),
    .spi_mosi  (pin_spi_mosi),
    .spi_sck   (pin_spi_sck),

    .boot      (boot_from_bootloader)
  );

endmodule
