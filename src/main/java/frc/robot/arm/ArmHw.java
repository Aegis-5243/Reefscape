package frc.robot.arm;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class ArmHw extends Arm {
    SparkMax armMotor;

    DigitalInput limitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH_PORT);

    double kP = Constants.ARM_kP;
    double kI = Constants.ARM_kI;
    double kD = Constants.ARM_kD;

    public ArmHw() {
        super();

        armMotor = new SparkMax(Constants.ARM_MOTOR_PORT, SparkMax.MotorType.kBrushless);
        armMotor.configure(
                new SparkMaxConfig()
                        .idleMode(SparkMaxConfig.IdleMode.kBrake)
                        .inverted(true)
                        .apply(new EncoderConfig()
                                .positionConversionFactor(Constants.ARM_POSITION_CONVERSION_FACTOR)
                                .velocityConversionFactor(Constants.ARM_VELOCITY_CONVERSION_FACTOR))
                        .apply(new SoftLimitConfig()
                                .reverseSoftLimit(Constants.ARM_MIN_POS)
                                .forwardSoftLimit(Constants.ARM_MAX_POS))
                        .apply(new ClosedLoopConfig()
                                .outputRange(-1, 1)
                                .pid(kP, kI, kD)
                                .apply(new MAXMotionConfig()
                                        .maxVelocity(Constants.ARM_MAX_VELOCITY)
                                        .maxAcceleration(Constants.ARM_MAX_ACCELERATION)))
                        .closedLoopRampRate(0.1),
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);

        setEncoderPosition(17);

        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        tab.addBoolean("Limit switch arm", this::getLimitSwitch);
    }

    public void periodic() {
        if (getLimitSwitch()) {
            setEncoderPosition(Constants.ARM_MIN_POS);
        }
    }

    @Override
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    @Override
    public double getMotorPosition() {
        return armMotor.getEncoder().getPosition();
    }

    @Override
    public double getAngle() {
        return getMotorPosition();
    }

    @Override
    public void setEncoderPosition(double degrees) {
        isPosition = false;
        armMotor.getEncoder().setPosition(degrees);
    }

    @Override
    public void setAngle(double degrees) {
        super.setAngle(degrees);
        armMotor.getClosedLoopController().setReference(degrees, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.50);

    }

    @Override
    public double getOutputCurrent() {
        return armMotor.getOutputCurrent();
    }

    @Override
    public double getOutputVoltage() {
        return armMotor.getAppliedOutput() * armMotor.getBusVoltage();
    }

    @Override
    public void stopArm() {
        isPosition = false;
        armMotor.stopMotor();
    }

    @Override
    public double getVelocity() {
        return armMotor.getEncoder().getVelocity();
    }

    @Override
    public void setVelocity(double velocity) {
        super.setVelocity(velocity);
        armMotor.getClosedLoopController().setReference(velocity, SparkMax.ControlType.kVelocity);
    }
}