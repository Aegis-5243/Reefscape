package frc.robot.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class ElevatorHw extends Elevator {
    SparkMax leftMotor;
    SparkMax rightMotor;

    DigitalInput limitSwitch = new DigitalInput(Constants.ELEVATOR_HALL_EFFECT_PORT);

    double kP = Constants.ELEVATOR_kP;
    double kI = Constants.ELEVATOR_kI;
    double kD = Constants.ELEVATOR_kD;
    double kF = 0;
    
    public ElevatorHw() {
        super();
        leftMotor = new SparkMax(Constants.ELEVATOR_LEFT_MOTOR_PORT, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.ELEVATOR_RIGHT_MOTOR_PORT, MotorType.kBrushless);

        SparkBaseConfig config = new SparkMaxConfig()
                .idleMode(SparkBaseConfig.IdleMode.kBrake);

        EncoderConfig encoderConfig = new EncoderConfig()
                .positionConversionFactor(Constants.ELEVATOR_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(Constants.ELEVATOR_VELOCITY_CONVERSION_FACTOR);

        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
                .pid(kP, kI, kD)
                .apply(new MAXMotionConfig()
                        .maxVelocity(Constants.ELEVATOR_MAX_VELOCITY));

        leftMotor.configure(
                new SparkMaxConfig()
                        .apply(config)
                        .apply(encoderConfig)
                        .apply(closedLoopConfig)
                        .inverted(true)
                        .disableFollowerMode()
                        .apply(new SoftLimitConfig()
                                .reverseSoftLimit(0)
                                .forwardSoftLimit(Constants.ELEVATOR_MAX_HEIGHT))
                        .closedLoopRampRate(0.3),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        rightMotor.configure(
                new SparkMaxConfig()
                        .apply(config)
                        .apply(encoderConfig)
                        .follow(leftMotor, true), // TODO: get proper inversions from REV hardware client
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        setEncoderPosition(0);
    }

    @Override
    public void setPosition(double distance) {
        super.setPosition(distance);
        leftMotor.getClosedLoopController().setReference(distance, ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.ELEVATOR_FF_VOLTAGE);
    }

    @Override
    public double getMotorPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    @Override
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    @Override
    public void setEncoderPosition(double position) {
        leftMotor.getEncoder().setPosition(position);
        rightMotor.getEncoder().setPosition(position);
    }

    public double getOutputCurrent() {
        return leftMotor.getOutputCurrent();
    }

    @Override
    public double getOutputVoltage() {
        return leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    }

    @Override
    protected double getVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        super.periodic();
        if (getLimitSwitch()) {

        }
    }

    @Override
    public void stopElevator() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public Command setPositionCmd(double position) {
        return run(() -> setPosition(position)).until(() -> Math.abs(getPosition() - position) < 1);
    }

    @Override
    public void setVelocity(double velocity) {
        leftMotor.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, Constants.ELEVATOR_FF_VOLTAGE);
    }
}
