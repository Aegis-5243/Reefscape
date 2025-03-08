// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {

    private static RollerSubsystem instance;

    public SparkMax roller;

    public TimeOfFlight laser;

    public SysIdRoutine sysId;

    public RelativeEncoder rollerEncoder;

    public SparkClosedLoopController rollerPIDController;

    /**
     * Creates a new RollerSubsystem
     */
    public RollerSubsystem() {

        this.roller = new SparkMax(Constants.ROLLER, MotorType.kBrushless);

        roller.configure(
                new SparkMaxConfig().idleMode(IdleMode.kBrake).disableFollowerMode().inverted(false)
                        .apply(new ClosedLoopConfig().p(Constants.ROLLER_kP).i(Constants.ROLLER_kI)
                                .d(Constants.ROLLER_kD).apply(new MAXMotionConfig().maxVelocity(90))),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.rollerEncoder = roller.getEncoder();

        this.rollerEncoder.setPosition(0);

        this.laser = new TimeOfFlight(Constants.TIME_OF_FLIGHT);

        this.sysId = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
                voltage -> {
                    roller.setVoltage(voltage);
                },
                log -> {
                    log.motor("roller")
                            .voltage(Units.Volts.of(roller.getBusVoltage()))
                            .linearPosition(Units.Inches
                                    .of(rollerEncoder.getPosition() * Constants.ROLLER_DIAMETER.in(Units.Inches) * Math.PI))
                            .linearVelocity(Units.Inches.per(Units.Minute)
                                    .of(rollerEncoder.getVelocity() * Constants.ROLLER_DIAMETER.in(Units.Inches)));
                },
                this));

        this.rollerPIDController = roller.getClosedLoopController();
        
        instance = this;
    }

    public static RollerSubsystem getInstance() {
        return instance;
    }

    /**
     * TeleOperated control for arm.
     * @deprecated Use roller.set() instead.
     */
    @Deprecated
    public void roll() {
        double speed = Constants.tertiaryStick.getThrottle();

        // System.out.println(speed);

        roller.set(speed);
    }

    /**
     * Set target velocity using PID.
     * @param velocity
     */
    public void setTargetVelocity(AngularVelocity velocity) {
        rollerPIDController.setReference(velocity.in(Units.RPM), ControlType.kVelocity);
    }

    /**
     * Set target position using PID.
     * @param dist
     */
    public void setTargetPosition(Angle dist) {
        rollerPIDController.setReference(dist.in(Units.Rotations), ControlType.kMAXMotionPositionControl);
    }

    /**
     * Detects if the rollers are practically still.
     * @return boolean, true if still.
     */
    public boolean isStill() {
        return Math.round(rollerEncoder.getVelocity() * 100) / 100.0 == 0;
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
        
        // System.out.println(laser.getRange());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
