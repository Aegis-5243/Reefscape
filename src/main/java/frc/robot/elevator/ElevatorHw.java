package frc.robot.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class ElevatorHw extends Elevator {
    SparkMax leftMotor;
    SparkMax rightMotor;

    DigitalInput limitSwitch = new DigitalInput(Constants.ELEVATOR_HALL_EFFECT_PORT);

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kF = 0;

    public ElevatorHw() {
        super();
        leftMotor = new SparkMax(Constants.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);

        SparkBaseConfig config = new SparkMaxConfig()
                .idleMode(SparkBaseConfig.IdleMode.kBrake);

        EncoderConfig encoderConfig = new EncoderConfig()
                .positionConversionFactor(Constants.ELEVATOR_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(Constants.ELEVATOR_VELOCITY_CONVERSION_FACTOR);

        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
                .pidf(kP, kI, kD, kF);

        leftMotor.configure(
                new SparkMaxConfig()
                        .apply(config)
                        .apply(encoderConfig)
                        .apply(closedLoopConfig)
                        .inverted(false) // TODO: get proper inversions from REV hardware client
                        .disableFollowerMode()
                        .apply(new SoftLimitConfig()
                                .reverseSoftLimit(0))
                        .closedLoopRampRate(0.5),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        rightMotor.configure(
                new SparkMaxConfig()
                        .apply(config)
                        .apply(encoderConfig)
                        .follow(leftMotor, true), // TODO: get proper inversions from REV hardware client
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    @Override
    public void setPosition(double distance) {
        super.setPosition(distance);
        leftMotor.getClosedLoopController().setReference(distance, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                Constants.ELEVATOR_STILL_PERCENT);
    }

    @Override
    public void setPower(double pct) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPower'");

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
    }

    @Override
    public void setVoltage(double volts) {
        leftMotor.setVoltage(volts);
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
}
