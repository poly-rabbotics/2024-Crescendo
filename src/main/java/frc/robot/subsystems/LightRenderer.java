package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.patterns.Rainbow;

/**
 * A class for rendering {@link LightPattern}s to lights strips on the robot.
 */
public class LightRenderer implements Runnable {
	private static final double DEFAULT_RAINBOW_SPEED = 50.0;

	private ColorUtils.BitArrangement[] bitArrangements = new ColorUtils.BitArrangement[0];
	private AddressableLED lightStrip;
	private AddressableLEDBuffer ledBuffer;
	private LightPattern pattern;
	private StatusedTimer timer = new StatusedTimer();
	boolean changedPattern;
	int patternIndex = 0;

	/**
	 * Sets the current light pattern and resets the internal timer.
	 */
	public void setPattern(LightPattern pattern) {
		timer.reset();
		this.pattern = pattern;
	}
	
	/**
	 * Sets the current light pattern and resets the internal timer.
	 */
	public void setPatternIfNotSameType(LightPattern pattern) {
		if (this.pattern.getClass() == pattern.getClass())
			return;
		
		timer.reset();
		this.pattern = pattern;
	}

	/**
	 * Sets the pattern if the current pattern determins the given one to be
	 * unequal, this usually means that any two patterns made with the same
	 * construction parameters will be equal and therefore not reset their
	 * animation when set.
	 */
	public void setIfNotEqual(LightPattern pattern) {
		if (this.pattern.isEqual(pattern)) {
			return;
		}

		timer.reset();
		this.pattern = pattern;
	}

	/**
	 * Stops this {@link LightRenderer}'s internal {@link StatusedTimer}.
	 */
	public void pause() {
		timer.stop();
	}

	/**
	 * Starts this {@link LightRenderer}'s internal {@link StatusedTimer}
	 */
	public void unpause() {
		timer.start();
	}

	/**
	 * Gets this {@link LightRenderer}'s internal {@link StatusedTimer}'s' status.
	 * 
	 * @return
	 * <code>true</code> if the timer is running. <code>false</code> if the timer is stopped.
	 */
	public boolean getPauseStatus() {
		return timer.getStatus();
	}

	/**
	 * Toggles this {@link LightRenderer}'s internal {@link StatusedTimer}'s' status.
	 * 
	 * @return
	 * <code>true</code> if the timer was started by this method. <code>false</code> if the timer was stopped.
	 */
	public boolean togglePauseStatus() {
		if (timer.getStatus())
			timer.stop();
		else
			timer.start();

		// Note that this timer.getStatus() call should always return the opposite value as was returned in the if statement
		// above, as it should have been toggled.
		return timer.getStatus();
	}

	/**
	 * Sets bit arrangements of colors in patterns by an array with equal 
	 * indices to the LED each bit arrangement wishes to address. If given an
	 * array that does not fill the whole strip, RGB will be assumed for all 
	 * remaining lights.
	 */
	public void setBitArrangements(ColorUtils.BitArrangement[] arrangements) {
		bitArrangements = arrangements;
	}

	/**
	 * Updates the LED light strip to account for time changes, calling this method continuously in a loop will allow patterns
	 * to appear fluid.
	 */
	public void run() {
		final Color[] colors = pattern.getPattern(timer.get());

		for (int led = 0; led < ledBuffer.getLength(); led++) {
			Color color = colors[led % pattern.getPatternLength()];

			if (led < bitArrangements.length) {
				color = ColorUtils.rearrangeColorToRGB(color, bitArrangements[led]);
			}

			ledBuffer.setLED(led, color);
		}

		if (pattern.getShouldResetTimer())
			timer.reset();

		lightStrip.setData(ledBuffer);
		lightStrip.start();
	}

	/**
	 * Creates a new {@link LightPattern}.
	 * 
	 * @param addressableLEDPort
	 * The port number of {@link AddressableLED} that this {@link LightRenderer} should 
	 * use.
	 * 
	 * @param bufferLength
	 * The length of the light strip in pixels.
	 */
	public LightRenderer(int addressableLEDPort, int bufferLength) {
		lightStrip = new AddressableLED(addressableLEDPort);
		ledBuffer = new AddressableLEDBuffer(bufferLength);
		lightStrip.setLength(bufferLength);
		pattern = new Rainbow(bufferLength, DEFAULT_RAINBOW_SPEED);
		lightStrip.start();
		timer.start();
	} 

	/**
	 * Creates a new {@link LightPattern}.
	 * 
	 * @param addressableLEDPort
	 * The port number of {@link AddressableLED} that this {@link LightRenderer} should 
	 * use.
	 * 
	 * @param bufferLength
	 * The length of the light strip in pixels.
	 * 
	 * @param pattern
	 * A {@link LightPattern} to be applied to following light renders in construction.
	 */
	public LightRenderer(int addressableLEDPort, int bufferLength, LightPattern pattern) {
		lightStrip = new AddressableLED(addressableLEDPort);
		ledBuffer = new AddressableLEDBuffer(bufferLength);
		lightStrip.setLength(bufferLength);
		this.pattern = pattern;
		timer.start();
	}
}
