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
import frc.robot.Constants;

public class IntakeHw extends Intake {
    public SparkMax roller;

    public TimeOfFlight laser1;
    public TimeOfFlight laser2;

    private double kP = Constants.ROLLER_kP;
    private double kI = Constants.ROLLER_kI;
    private double kD = Constants.ROLLER_kD;

    public IntakeHw() {
        roller = new SparkMax(Constants.ROLLER_PORT, MotorType.kBrushless);

        roller.configure(
                new SparkMaxConfig()
                        .idleMode(IdleMode.kBrake)
                        .disableFollowerMode()
                        .inverted(true)
                        .apply(new EncoderConfig()
                                .positionConversionFactor(Constants.ROLLER_POSITION_CONVERSION_FACTOR)
                                .velocityConversionFactor(Constants.ROLLER_VELOCITY_CONVERSION_FACTOR))
                        .apply(new ClosedLoopConfig()
                                .pid(kP, kI, kD)
                                .apply(new MAXMotionConfig().maxVelocity(90)))
                        .openLoopRampRate(0.2)
                        .closedLoopRampRate(0.3),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        roller.getEncoder().setPosition(0);

        laser1 = new TimeOfFlight(Constants.ROLLER_TIME_OF_FLIGHT_1_PORT);
        laser2 = new TimeOfFlight(Constants.ROLLER_TIME_OF_FLIGHT_2_PORT);

        laser1.setRangingMode(RangingMode.Short, 24);
        laser2.setRangingMode(RangingMode.Short, 24);

        Shuffleboard.getTab("Intake").addDouble("TOF 1 dist", () -> laser1.getRange());
        Shuffleboard.getTab("Intake").addDouble("TOF 2 dist", () -> laser2.getRange());

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
    public double getOutputVoltage() {
        return roller.getAppliedOutput() * roller.getBusVoltage();
    }

    @Override
    public void updateSensors() {
        if (detectingCoral1()) {
            if (detectingCoral2()) {
                currentCoralState = CoralStates.SAFE;
            } else {
                currentCoralState = CoralStates.INWARD;
            }
        } else {
            if (detectingCoral2()) {
                currentCoralState = CoralStates.OUTWARD;
            } else {
                currentCoralState = CoralStates.NONE;
            }
        }
    }

    // 0.3 is 20 inches per second
    public void setPower(double power) {
        super.setPower(power);
        roller.set(power);
    }

    @Override
    public void stopIntake() {
        isPosition = false;
        roller.set(0);
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
    /** Depriciate once 2nd TOF is added */
    public boolean hasCoral() {
        return detectingCoral1();
    }

    @Override
    public boolean detectingCoral1() {
        return laser1.getRange() < Constants.ROLLER_CORAL_DETECTION_THRESHOLD_1;
    }

    @Override
    public boolean detectingCoral2() {
        return laser2.getRange() < Constants.ROLLER_CORAL_DETECTION_THRESHOLD_2;
    }
}
