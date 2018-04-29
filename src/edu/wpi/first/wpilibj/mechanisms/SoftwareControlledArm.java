package edu.wpi.first.wpilibj.mechanisms;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.sensing.PositionAndVelocitySource;
import edu.wpi.first.wpilibj.controlledmotors.SoftwareControlledMotor;
import edu.wpi.first.wpilibj.sensing.VelocitySource;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

public class SoftwareControlledArm {

    private final SoftwareControlledMotor controlledMotor;
    private final double gravityCompensationTorquePercentageAtLevel;

    public SoftwareControlledArm(SpeedController voltageController,
                                 Encoder encoder,
                                 double gravityCompensationTorquePercentageAtLevel,
                                 double degreesPerSensorUnit,
                                 double sensorValueAtZeroDegrees,
                                 double maxSpeedInSensorUnitsPerSecond) {

        this(voltageController,
                PositionAndVelocitySource.from(
                        encoder::getDistance,
                        encoder::getRate),
                gravityCompensationTorquePercentageAtLevel,
                degreesPerSensorUnit,
                sensorValueAtZeroDegrees,
                maxSpeedInSensorUnitsPerSecond);
    }

    public SoftwareControlledArm(SpeedController voltageController,
                                 Potentiometer potentiometer,
                                 double gravityCompensationTorquePercentageAtLevel,
                                 double degreesPerSensorUnit,
                                 double sensorValueAtZeroDegrees,
                                 double maxSpeedInSensorUnitsPerSecond) {

        this(voltageController,
                PositionAndVelocitySource.from(
                        potentiometer::get,
                        VelocitySource.differentiatePosition(potentiometer::get)),
                gravityCompensationTorquePercentageAtLevel,
                degreesPerSensorUnit,
                sensorValueAtZeroDegrees,
                maxSpeedInSensorUnitsPerSecond);
    }

    public SoftwareControlledArm(SpeedController voltageController,
                                 PositionAndVelocitySource stateSource,
                                 double gravityCompensationTorquePercentageAtLevel,
                                 double degreesPerSensorUnit,
                                 double sensorValueAtZeroDegrees,
                                 double maxSpeedInSensorUnitsPerSecond) {

        this.gravityCompensationTorquePercentageAtLevel =
                gravityCompensationTorquePercentageAtLevel;

        final PositionAndVelocitySource levelAtZeroDegreesSource =
                PositionAndVelocitySource.corrected(stateSource,
                        -sensorValueAtZeroDegrees,
                        degreesPerSensorUnit);

        this.controlledMotor = SoftwareControlledMotor.builder()
                .withSpeedController(voltageController)
                .withPositionSource(levelAtZeroDegreesSource)
                .withVelocitySource(levelAtZeroDegreesSource)
                .positionUnitsPerSensorUnit(1)
                .withMaxSpeedInSensorUnitsPerSecond(maxSpeedInSensorUnitsPerSecond)
                .withTorqueOffsetComputer(this::computeGravityCompensationTorquePercent)
                .build();
    }

    public void setPositionDegrees(double positionDegrees) {
        controlledMotor.setDesiredAbsolutePosition(positionDegrees);
    }

    public double getPositionDegrees() {
        return controlledMotor.getPosition();
    }

    private double computeGravityCompensationTorquePercent() {
        double angleRad = Math.toRadians(getPositionDegrees());

        return Math.cos(angleRad) * gravityCompensationTorquePercentageAtLevel;
    }
}