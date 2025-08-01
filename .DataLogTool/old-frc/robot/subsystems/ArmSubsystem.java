// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    
    private static ArmSubsystem instance;

    public SparkMax arm;

    public SysIdRoutine sysId;

    public RelativeEncoder armEncoder;

    public RelativeEncoder armAltEncoder;

    public SparkClosedLoopController armPIDController;
    public PIDController armWPIPIDcontroller;

    public double setpoint;

    public DigitalInput limitSwitch;

    /**
     * Creates a new ArmSubsystem
     */
    public ArmSubsystem() {
        this.arm = new SparkMax(Constants.ARM, MotorType.kBrushless);

        // arm.configure(
        //         new SparkMaxConfig().idleMode(IdleMode.kBrake).disableFollowerMode().inverted(false)
        //                 .apply(new SoftLimitConfig().reverseSoftLimit(0))
        //                 .apply(new ClosedLoopConfig().p(Constants.ARM_kP).i(Constants.ARM_kI)
        //                         .d(Constants.ARM_kD).feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        //                         .apply(new MAXMotionConfig().maxVelocity(Constants.ARM_MAX_VELOCITY.in(Units.RPM))
        //                                 .maxAcceleration(
        //                                         Constants.ARM_MAX_ACCELERATION.in(Units.RPM.per(Units.Second))))),
        //         ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.armEncoder = arm.getEncoder();

        this.armEncoder.setPosition(0);

        this.armPIDController = arm.getClosedLoopController();

        this.limitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH);

        this.setpoint = 0;

        this.armWPIPIDcontroller = new PIDController(Constants.ARM_kP, Constants.ARM_kI, Constants.ARM_kD);

        this.armAltEncoder = arm.getAlternateEncoder();

        Shuffleboard.getTab("PID test").add("ARM PID", armWPIPIDcontroller);

        instance = this;

        Shuffleboard.getTab("Arm").addDouble("Arm Current Position", () -> armEncoder.getPosition());
        Shuffleboard.getTab("Arm").addDouble("Arm Target Position", () -> this.setpoint);
        Shuffleboard.getTab("Arm").addDouble("Arm Velocity", () -> armEncoder.getVelocity());
        Shuffleboard.getTab("Arm").addDouble("Arm Current", () -> arm.getOutputCurrent());
    }

    public static ArmSubsystem getInstance() {
        return instance;
    }

    /**
     * TeleOperated control for arm.
     * @deprecated Use applySpeed() instead.
     */
    @Deprecated
    public void arm() {
        double speed = Constants.tertiaryStick.getY();

        // replace with applySpeed after proper testing and wiring.
        arm.set(speed);
    }

    /**
     * Applies speed to the arm.
     * @param speed percentage of how fast the arm should travel. -1 <= speed <= 1
     */
    public void applySpeed(double speed) {

        speed = (armEncoder.getPosition() <= Constants.ARM_MIN_POS.in(Units.Rotations) && speed < 0)
                || (armEncoder.getPosition()
                        * Constants.ARM_GEAR_RATIO >= Constants.ARM_MAX_POS.in(Units.Rotations)
                        && speed > 0) ? 0 : speed; // genuinely what the fuck is this for readability

        arm.set(speed);
    }

    /**
     * Check if limit switch is triggered.
     * <p>If so, zero encoder.
     */
    public void checklimitSwitch() {
        // if (!limitSwitch.get()) {
        //     armEncoder.setPosition(0);
        // }
    }

    /**
     * Set target position for PID.
     * @param rotations Target position in rotations
     */
    public void setTargetPosition(double rotations) {
        armPIDController.setReference(rotations, ControlType.kPosition);
        setpoint = rotations;
    }

    /**
     * Detects if arm is practically still.
     * @return boolean, true if still.
     */
    public boolean isStill() {
        return Math.round(armEncoder.getVelocity() * 100) / 100.0 == 0;
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    public boolean isStalled() {
        return arm.getOutputCurrent() > Constants.ARM_STALL_CURRENT;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // if (isStalled()) {
        //     System.out.println("Arm is stalled! Current: " + arm.getOutputCurrent());
            
        //     arm.set(0);
        // }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
