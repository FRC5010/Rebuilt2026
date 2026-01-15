// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.config.ConfigConstants;

public class LEDStrip extends GenericSubsystem {
  private int kLength = 0;
  private LEDPattern defaultPattern = LEDPattern.solid(Color.kGreen);

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public static class Segment {
    public LEDPattern pattern;
    public AddressableLEDBufferView view;
    public boolean active = false;
    public int order = 0;

    /**
     * Creates a new ActivePattern object.
     *
     * @return the new ActivePattern object.
     */
    public static Segment create() {
      Segment ap = new Segment();
      return ap;
    }

    /**
     * Sets the pattern of the active pattern.
     *
     * @param pattern the pattern to set
     * @return the active pattern object
     */
    public Segment setPattern(LEDPattern pattern) {
      this.pattern = pattern;
      return this;
    }

    /**
     * Sets the view of the active pattern.
     *
     * @param view the view to set
     * @return the active pattern object
     */
    public Segment setView(AddressableLEDBufferView view) {
      this.view = view;
      return this;
    }

    /**
     * Sets whether the active pattern is active or not.
     *
     * @param active whether the active pattern is active or not
     * @return the active pattern object
     */
    public Segment setActive(boolean active) {
      this.active = active;
      return this;
    }

    /**
     * Sets the order of the active pattern.
     *
     * @param order the order to set
     * @return the active pattern object
     */
    public Segment setOrder(int order) {
      this.order = order;
      return this;
    }

    /**
     * Gets the pattern of the active pattern.
     *
     * @return the pattern of the active pattern
     */
    public LEDPattern getPattern() {
      return pattern;
    }

    /**
     * Gets the view of the active pattern.
     *
     * @return the view of the active pattern
     */
    public AddressableLEDBufferView getView() {
      return view;
    }

    /**
     * Gets whether the active pattern is active or not.
     *
     * @return whether the active pattern is active or not
     */
    public boolean isActive() {
      return active;
    }

    /**
     * Gets the order of the active pattern.
     *
     * @return the order of the active pattern
     */
    public int getOrder() {
      return order;
    }
  }

  private static final Map<String, Segment> segments = new HashMap<>();
  private static LEDStrip instance;

  /**
   * Creates a new instance of the LEDStrip class if one does not already exist.
   *
   * @param kPort the port the LED strip is connected to
   * @param length the length of the LED strip in number of LEDs
   */
  public static void createInstance(int kPort, int length) {
    if (instance == null) {
      instance = new LEDStrip(kPort, length);
    }
  }

  /**
   * Creates a new LEDStrip.
   *
   * @param kPort the port the LED strip is connected to
   * @param length the length of the LED strip in number of LEDs
   * @param physicalLength the physical length of the LED strip
   */
  private LEDStrip(int kPort, int length) {
    this.kLength = length;

    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    segments.put(
        ConfigConstants.ALL_LEDS,
        Segment.create()
            .setView(m_buffer.createView(0, kLength - 1))
            .setPattern(defaultPattern)
            .setActive(true));
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(runPattern().ignoringDisable(true));
  }

  /**
   * Sets the default pattern to run on the entire LED strip when the LED subsystem is not running a
   * command.
   *
   * @param pattern the LED pattern to run by default
   */
  public void setFullPattern(LEDPattern pattern) {
    this.defaultPattern = pattern;
  }

  /**
   * Gets the length of the LED strip in number of LEDs.
   *
   * @return the length of the LED strip
   */
  public int getLength() {
    return kLength;
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @return a command that runs the pattern
   */
  public Command runPattern() {
    return run(
        () -> {
          segments.values().stream()
              .sorted(Comparator.comparingInt(Segment::getOrder))
              .forEach(
                  ap -> {
                    if (ap.isActive()) {
                      ap.getPattern().applyTo(ap.getView());
                    }
                  });
        });
  }

  /** Periodically sends the latest LED color data to the LED strip for it to display. */
  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to
    // display
    m_led.setData(m_buffer);
  }

  /**
   * Sets whether the pattern with the given name is active or not. If the name does not correspond
   * to a pattern, this method does nothing.
   *
   * @param name The name of the pattern to set whether it is active or not
   * @param active Whether the pattern is active or not
   */
  public static void setSegmentActive(String name, boolean active) {
    if (segments.containsKey(name)) {
      segments.get(name).setActive(active);
    }
  }

  /**
   * Adds a pattern to the LED strip with the given name, starting at the given index and spanning
   * the given length. The pattern is active by default. If the name already corresponds to a
   * pattern, this method does nothing.
   *
   * @param name The name of the pattern to add
   * @param pattern The pattern to add
   * @param startIndex The starting index of the pattern
   * @param length The length of the pattern
   */
  public static void addSegment(String name, LEDPattern pattern, int startIndex, int length) {
    if (null == instance) {
      return;
    }
    AddressableLEDBufferView view = instance.m_buffer.createView(startIndex, length);
    segments.put(name, Segment.create().setView(view).setPattern(pattern).setActive(true));
  }

  /**
   * Adds a pattern to the LED strip with the given name, starting at the given index and spanning
   * the given length. If the name already corresponds to a pattern, this method does nothing.
   *
   * @param name The name of the pattern to add
   * @param startIndex The starting index of the pattern
   * @param length The length of the pattern
   */
  public static void addSegment(String name, int startIndex, int length) {
    if (null == instance) {
      return;
    }
    AddressableLEDBufferView view = instance.m_buffer.createView(startIndex, length);
    segments.put(name, Segment.create().setView(view).setPattern(LEDPattern.kOff).setActive(false));
  }

  /**
   * Adds a pattern to the LED strip with the given name, starting at the given index and spanning
   * the given length. The pattern is active by default. If the name already corresponds to a
   * pattern, this method does nothing.
   *
   * @param name The name of the pattern to add
   * @param startIndex The starting index of the pattern
   * @param length The length of the pattern
   * @param order The order in which the pattern will be displayed. Lower order patterns will be
   *     displayed before higher order patterns.
   */
  public static void addSegment(String name, int startIndex, int length, int order) {
    if (null == instance) {
      return;
    }
    AddressableLEDBufferView view = instance.m_buffer.createView(startIndex, length);
    segments.put(
        name,
        Segment.create()
            .setView(view)
            .setPattern(LEDPattern.kOff)
            .setActive(false)
            .setOrder(order));
  }

  /**
   * Removes the pattern with the given name from the LED strip. If no pattern with the given name
   * exists, this method does nothing.
   *
   * @param name The name of the pattern to remove
   */
  public static void removeSegment(String name) {
    segments.remove(name);
  }

  /**
   * Clears all active patterns on the LED strip. This method is useful for completely clearing the
   * LED strip of all patterns before running a new command.
   */
  public static void clearSegments() {
    segments.clear();
  }

  /**
   * Sets the default command to be run by the LED subsystem. If the LED subsystem has not been
   * initialized yet, this method does nothing.
   *
   * @param defaultCommand the default command to be run by the LED subsystem
   */
  public static void setCommand(Command defaultCommand) {
    if (null != instance) {
      instance.setDefaultCommand(defaultCommand);
    }
  }

  /**
   * Updates the pattern of the active pattern with the given name. If no active pattern with the
   * given name exists, this method does nothing.
   *
   * @param name The name of the active pattern to update
   * @param pattern The new pattern for the active pattern
   */
  public static void changeSegmentPattern(String name, LEDPattern pattern) {
    if (segments.containsKey(name)) {
      segments.get(name).setPattern(pattern);
    }
  }

  /**
   * Returns a new LEDPattern that sets all LEDs to a rainbow color, and scrolls the pattern at the
   * given speed.
   *
   * @param percentScrollingSpeed the speed at which to scroll the pattern, as a percentage of the
   *     pattern's length per second
   * @return the new LEDPattern
   */
  public static LEDPattern getRainbowPattern(double percentScrollingSpeed) {
    return LEDPattern.rainbow(255, 255)
        .scrollAtRelativeSpeed(Percent.per(Second).of(percentScrollingSpeed));
  }

  /**
   * Returns a new LEDPattern that sets all LEDs to the given solid color.
   *
   * @param color The color to set all LEDs to
   * @return The new solid color pattern
   */
  public static LEDPattern getSolidPattern(Color color) {
    return LEDPattern.solid(color);
  }

  /**
   * Returns a new LEDPattern that masks the given base pattern with a band of given visibility and
   * scrolling speed. The mask has a band of white (visible) that is percentVisible of the pattern's
   * length wide, centered at percentVisible of the pattern's length from the start of the pattern.
   * The rest of the mask is black (invisible). The resulting pattern is then scrolled at the given
   * speed.
   *
   * @param basePattern the base pattern to mask
   * @param percentVisible the percentage of the pattern's length that is visible
   * @param percentScrollingSpeed the speed at which to scroll the pattern, as a percentage of the
   *     pattern's length per second
   * @return the new masked and scrolling pattern
   */
  public static LEDPattern getMaskedPattern(
      LEDPattern basePattern, double percentVisible, double percentScrollingSpeed) {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, percentVisible, Color.kBlack);
    LEDPattern mask =
        LEDPattern.steps(maskSteps)
            .scrollAtRelativeSpeed(Percent.per(Second).of(percentScrollingSpeed));
    return basePattern.mask(mask);
  }

  /**
   * Returns a new LEDPattern that masks the given base pattern with a band of given center and
   * width, and scrolls the resulting pattern at the given speed.
   *
   * @param basePattern the base pattern to mask
   * @param bandCenter the center of the band, as a percentage of the pattern's length
   * @param percentWidth the width of the band, as a percentage of the pattern's length
   * @param percentScrollingSpeed the speed at which to scroll the pattern, as a percentage of the
   *     pattern's length per second
   * @return the new masked and scrolling pattern
   */
  public static LEDPattern getBand(
      LEDPattern basePattern,
      double bandCenter,
      double percentWidth,
      double percentScrollingSpeed) {
    Map<Double, Color> maskSteps =
        Map.of(
            0.0,
            Color.kBlack,
            bandCenter - (percentWidth / 2),
            Color.kWhite,
            bandCenter - (percentWidth / 2),
            Color.kBlack);
    LEDPattern mask =
        LEDPattern.steps(maskSteps)
            .scrollAtRelativeSpeed(Percent.per(Second).of(percentScrollingSpeed));
    return basePattern.mask(mask);
  }

  /**
   * Returns a new LEDPattern that blinks the given base pattern at the given interval.
   *
   * @param basePattern the base pattern to blink
   * @param blinkInterval the interval at which to blink the pattern
   * @return the new blinking pattern
   */
  public static LEDPattern getBlinkingPattern(LEDPattern basePattern, Time blinkInterval) {
    return basePattern.blink(blinkInterval);
  }
}
