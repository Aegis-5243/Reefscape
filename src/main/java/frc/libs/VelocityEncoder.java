package frc.libs;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class VelocityEncoder extends Encoder {
    private final Timer time = new Timer();
    private final double RADIANS_PER_PULSE;
    private final double METERS_PER_PULSE;
    
    /**
     * VelocityEncoder constructor. Construct an Encoder given a and b channels.
     * This is a wrapper class. You can achieve the same results with Encoder.getRate(). This class, however, helps ensure type safety.
     *
     * Assumes motor is a drive motor and encoder is a REV Through Bore Encoder.
     * 
     * The encoder will start counting immediately.
     *
     * @param channelA The a channel digital input channel.
     * @param channelB The b channel digital input channel.
     * @param wheelDiameter The diameter of the attached wheel
     */
    public VelocityEncoder(int channelA, int channelB) {
        this(channelA, channelB, Constants.WHEEL_DIAMETER, Constants.THROUGH_BORE_RESOLUTION);
    }

    /**
     * VelocityEncoder constructor. Construct an Encoder given a and b channels as well as the wheel diameter of the wheel attached.
     * This is a wrapper class. You can achieve the same results with Encoder.getRate(). This class, however, helps ensure type safety.
     *
     * Assumes the encoder is a REV Through Bore Encoder.
     * 
     * The encoder will start counting immediately.
     *
     * @param channelA The a channel digital input channel.
     * @param channelB The b channel digital input channel.
     * @param wheelDiameter The diameter of the attached wheel
     */
    public VelocityEncoder(int channelA, int channelB, Distance wheelDiameter) {
        this(channelA, channelB, wheelDiameter, Constants.THROUGH_BORE_RESOLUTION);
    }

    /**
     * VelocityEncoder constructor. Construct an Encoder given a and b channels as well as the wheel diameter of the wheel attached.
     * This is a wrapper class. You can achieve the same results with Encoder.getRate(). This class, however, helps ensure type safety.
     *
     * The encoder will start counting immediately.
     *
     * @param channelA The a channel digital input channel.
     * @param channelB The b channel digital input channel.
     * @param wheelDiameter The diameter of the attached wheel
     * @param encoderTicksPerRevolution The encoder's counts per revolution
     */
    public VelocityEncoder(int channelA, int channelB, Distance wheelDiameter, double encoderPulsesPerRevolution) {
        super(channelA, channelB);
        
        this.time.restart();
        this.RADIANS_PER_PULSE = 2 * Math.PI / encoderPulsesPerRevolution;
        this.METERS_PER_PULSE = Math.PI * wheelDiameter.in(Units.Meters) / encoderPulsesPerRevolution;
    }

    /**
     * Update linear and angular velocity variables
     * @deprecated Unnecessary. Will update without call.
     */
    @Deprecated
    public void update() {
        
        // double currPos = (this.get() / this.TICKS_PER_REVOLUTION) * (Math.PI * this.WHEEL_DIAMETER);
        // double currAng = (this.get() / this.TICKS_PER_REVOLUTION) * 2 * Math.PI;
        // double currTime = time.get();

        // linearVelocity = Units.MetersPerSecond.of((currPos - lastPos) / (currTime - lastTime));
        // angVelocity = Units.RadiansPerSecond.of((currAng - lastAngle) / (currTime - lastTime));
        
        // lastTime = currTime;
        // lastPos = currPos;
        // lastAngle = currAng;
    }

    /**
     * Get linear velocity of the motor with a wheel attatched.
     * @return linear velocity of the motor with a wheel.
     */
    public LinearVelocity getLinearVelocity() {
        this.setDistancePerPulse(METERS_PER_PULSE);
        return Units.MetersPerSecond.of(this.getRate());
    }

    /**
     * Get angular velocity of the encoder.
     * @return angular velocity of an encoder.
     */
    public AngularVelocity getAngularVelocity() {
        this.setDistancePerPulse(RADIANS_PER_PULSE);
        return Units.RadiansPerSecond.of(this.getRate());
    }

    public Distance getDist() {
        this.setDistancePerPulse(METERS_PER_PULSE);
        return Units.Meters.of(this.getDistance());
    }

    
    public Angle getAngle() {
        this.setDistancePerPulse(RADIANS_PER_PULSE);
        return Units.Radians.of(this.getDistance());
    }
}