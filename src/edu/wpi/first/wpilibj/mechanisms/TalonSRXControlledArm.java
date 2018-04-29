package edu.wpi.first.wpilibj.mechanisms;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.controlledmotors.TalonSRXControlledMotor;

public class TalonSRXControlledArm {

    private final TalonSRXControlledMotor controlledMotor;
    private final double gravityCompensationVoltagePercentageAtLevel;

    public TalonSRXControlledArm(int channel,
                                 FeedbackDevice feedbackDevice,
                                 double gravityCompensationVoltagePercentageAtLevel,
                                 double degreesPerSensorUnit,
                                 double sensorValueAtZeroDegrees,
                                 double maxSpeedInSensorUnitsPerSecond) {

        this.gravityCompensationVoltagePercentageAtLevel =
                gravityCompensationVoltagePercentageAtLevel;

        this.controlledMotor = new TalonSRXControlledMotor(
                channel,
                feedbackDevice,
                this::computeGravityCompensationVoltagePercent,
                -sensorValueAtZeroDegrees,
                degreesPerSensorUnit,
                maxSpeedInSensorUnitsPerSecond
        );
    }

    public void  setPositionDegrees(double positionDegrees) {
        controlledMotor.setDesiredAbsolutePosition(positionDegrees);
    }

    public double getPositionDegrees() {
        return controlledMotor.getPosition();
    }

    private double computeGravityCompensationVoltagePercent() {
        double angleRad = Math.toRadians(getPositionDegrees());

        return Math.cos(angleRad) * gravityCompensationVoltagePercentageAtLevel;
    }
}