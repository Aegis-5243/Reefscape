package frc.robot.lib;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.Utilities;

public class VelocityEncoder extends Encoder {
    private Timer time = new Timer();
    private double wheelDiameter;
    private double lastTime;
    private double lastPos;
    private double lastAngle;
    private LinearVelocity linearVelocity;
    private AngularVelocity angVelocity;
    

    /**
     * VelocityEncoder constructor. Construct an Encoder given a and b channels as well as the wheel diameter of the wheel attached.
     *
     * The encoder will start counting immediately.
     *
     * @param channelA The a channel digital input channel.
     * @param channelB The b channel digital input channel.
     * @param wheelDiameter The diameter of the attached wheel
     */
    public VelocityEncoder(int channelA, int channelB, Distance wheelDiameter) {
        super(channelA, channelB);

        this.time.restart();
        this.wheelDiameter = wheelDiameter.in(Meters);
        this.lastTime = this.time.get();
        this.lastPos = (this.get() / Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION) * (Math.PI * this.wheelDiameter);
        this.lastAngle = (this.get() / Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION) * 2 * Math.PI;
    }

    /**
     * Update linear and angular velocity variables
     */
    public void update() {
        double currPos = (this.get() / Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION) * (Math.PI * this.wheelDiameter);
        double currAng = (this.get() / Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION) * 2 * Math.PI;
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