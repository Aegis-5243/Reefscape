package frc.libs;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class VelocityEncoder extends Encoder {
    private final Timer time = new Timer();
    private final double WHEEL_DIAMETER;
    private final double TICKS_PER_REVOLUTION;
    private double lastTime;
    private double lastPos;
    private double lastAngle;
    private LinearVelocity linearVelocity;
    private AngularVelocity angVelocity;
    
    /**
     * VelocityEncoder constructor. Construct an Encoder given a and b channels.
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
        this(channelA, channelB, Constants.WHEEL_DIAMETER, Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION);
    }

    /**
     * VelocityEncoder constructor. Construct an Encoder given a and b channels as well as the wheel diameter of the wheel attached.
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
        this(channelA, channelB, wheelDiameter, Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION);
    }

    /**
     * VelocityEncoder constructor. Construct an Encoder given a and b channels as well as the wheel diameter of the wheel attached.
     *
     * The encoder will start counting immediately.
     *
     * @param channelA The a channel digital input channel.
     * @param channelB The b channel digital input channel.
     * @param wheelDiameter The diameter of the attached wheel
     * @param encoderTicksPerRevolution The encoder's counts per revolution
     */
    public VelocityEncoder(int channelA, int channelB, Distance wheelDiameter, double encoderTicksPerRevolution) {
        super(channelA, channelB);
        
        this.time.restart();
        this.WHEEL_DIAMETER = wheelDiameter.in(Units.Meters);
        this.TICKS_PER_REVOLUTION = encoderTicksPerRevolution;
        this.lastTime = this.time.get();
        this.lastPos = (this.get() / this.TICKS_PER_REVOLUTION) * (Math.PI * this.WHEEL_DIAMETER);
        this.lastAngle = (this.get() / this.TICKS_PER_REVOLUTION) * 2 * Math.PI;
    }

    /**
     * Update linear and angular velocity variables
     */
    public void update() {
        double currPos = (this.get() / this.TICKS_PER_REVOLUTION) * (Math.PI * this.WHEEL_DIAMETER);
        double currAng = (this.get() / this.TICKS_PER_REVOLUTION) * 2 * Math.PI;
        double currTime = time.get();

        linearVelocity = Units.MetersPerSecond.of((currPos - lastPos) / (currTime - lastTime));
        angVelocity = Units.RadiansPerSecond.of((currAng - lastAngle) / (currTime - lastTime));
        
        lastTime = currTime;
        lastPos = currPos;
        lastAngle = currAng;
    }

    /**
     * Get linear velocity of the motor with a wheel attatched.
     * @return linear velocity of the motor with a wheel.
     */
    public LinearVelocity getLinearVelocity() {
        update();
        return linearVelocity;
    }

    /**
     * Get angular velocity of the encoder.
     * @return angular velocity of an encoder.
     */
    public AngularVelocity getAngularVelocity() {
        update();
        return angVelocity;
    }
}