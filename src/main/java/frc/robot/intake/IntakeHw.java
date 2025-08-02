package frc.robot.intake;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class IntakeHw extends Intake {
    public SparkMax roller;

    public TimeOfFlight laser;

    private double kP = Constants.ROLLER_kP;
    private double kI = Constants.ROLLER_kI;
    private double kD = Constants.ROLLER_kD;

    public IntakeHw() {
        roller = new SparkMax(Constants.ROLLER_PORT, MotorType.kBrushless);

        roller.configure(
                new SparkMaxConfig()
                        .idleMode(IdleMode.kBrake)
                        .disableFollowerMode()
                        .inverted(false)
                        .apply(new EncoderConfig()
                                .positionConversionFactor(Constants.ROLLER_POSITION_CONVERSION_FACTOR)
                                .velocityConversionFactor(Constants.ROLLER_VELOCITY_CONVERSION_FACTOR))
                        .apply(new ClosedLoopConfig()
                                .pid(kP, kI, kD)
                                .apply(new MAXMotionConfig().maxVelocity(90))),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        roller.getEncoder().setPosition(0);

        laser = new TimeOfFlight(Constants.ROLLER_TIME_OF_FLIGHT_PORT);
        
        laser.setRangingMode(RangingMode.Short, 24);

        
    }

    @Override
    public double getVelocity() {
        return roller.getEncoder().getVelocity();
    }

    @Override
    public void setVelocity(double speed) {
        super.setVelocity(speed);
        roller.getClosedLoopController().setReference(speed, SparkMax.ControlType.kVelocity);
    }

    @Override
    public double getOutputCurrent() {
        return roller.getOutputCurrent();
    }

    @Override
    public void updateSensors() {
        laser.getRange();
    }

    @Override
    public double getPosition() {
        return roller.getEncoder().getPosition();
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(position);
        roller.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
    }

    @Override
    public boolean detectingCoral() {
        return laser.getRange() < Constants.ROLLER_CORAL_DETECTION_THRESHOLD;
    }
}
