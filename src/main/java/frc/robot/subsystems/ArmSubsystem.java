// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private static ArmSubsystem instance;

    public SparkMax arm;

    public SysIdRoutine sysId;

    public RelativeEncoder armEncoder;

    public ArmFeedforward armFeedforward;

    public SparkClosedLoopController armPIDController;

	// public static ComplexWidget PIDWidget;

    public ArmSubsystem() {

        this.arm = new SparkMax(Constants.ARM, MotorType.kBrushless);

        arm.configure(
                new SparkMaxConfig().idleMode(IdleMode.kBrake).disableFollowerMode().inverted(false)
                        .apply(new SoftLimitConfig().reverseSoftLimit(0))
                        .apply(new ClosedLoopConfig().p(Constants.ARM_kP).i(Constants.ARM_kI)
                                .d(Constants.ARM_kD)
                                .apply(new MAXMotionConfig().maxVelocity(Constants.ARM_MAX_VELOCITY.in(Units.RPM))
                                        .maxAcceleration(
                                                Constants.ARM_MAX_ACCELERATION.in(Units.RPM.per(Units.Second))))),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.armEncoder = arm.getEncoder();

        this.armEncoder.setPosition(0);

        this.armFeedforward = new ArmFeedforward(Constants.ARM_kS, Constants.ARM_kG,
                Constants.ARM_kV, Constants.ARM_kA);

        this.armPIDController = arm.getClosedLoopController();

        instance = this;
    }

    public static ArmSubsystem getInstance() {
        return instance;
    }

    public void arm() {
        double speed = Constants.tertiaryStick.getY();

        // replace with applySpeed after proper testing and wiring.
        arm.set(speed);
    }

    public void applySpeed(double speed) {

        speed = (armEncoder.getPosition() <= Constants.ARM_MIN_POS.in(Units.Rotations) && speed < 0)
                || (armEncoder.getPosition()
                        * Constants.ARM_GEAR_RATIO >= Constants.ARM_MAX_POS.in(Units.Rotations)
                        && speed > 0) ? 0 : speed;

        arm.set(speed);
    }

    public void setTargetPosition(double rotations) {
        armPIDController.setReference(rotations, ControlType.kMAXMotionPositionControl);
    }

    public boolean isStill() {
        return armEncoder.getVelocity() == 0;
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
