package edu.wpi.first.wpilibj.mechanisms;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.controlledmotors.TalonSRXControlledMotor;

public class TalonSRXControlledElevator {

    private final TalonSRXControlledMotor controlledMotor;
    private final double gravityCompensationVoltagePercentageAtLevel;

    public TalonSRXControlledElevator(int channel,
                                      FeedbackDevice feedbackDevice,
                                      double gravityCompensationVoltagePercentageAtLevel,
                                      double degreesPerSensorUnit,
                                      double sensorValueAtZero,
                                      double maxSpeedInSensorUnitsPerSecond) {

        this.gravityCompensationVoltagePercentageAtLevel =
                gravityCompensationVoltagePercentageAtLevel;

        this.controlledMotor = new TalonSRXControlledMotor(
                channel,
                feedbackDevice,
                this::computeGravityCompensationVoltagePercent,
                -sensorValueAtZero,
                degreesPerSensorUnit,
                maxSpeedInSensorUnitsPerSecond
        );
    }

    private void setPosition(double positionControllerUnits) {
        controlledMotor.setDesiredAbsolutePosition(positionControllerUnits);
    }

    private double computeGravityCompensationVoltagePercent() {
        return gravityCompensationVoltagePercentageAtLevel;
    }
}