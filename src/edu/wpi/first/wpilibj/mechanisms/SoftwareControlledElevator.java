package edu.wpi.first.wpilibj.mechanisms;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.sensing.PositionAndVelocitySource;
import edu.wpi.first.wpilibj.controlledmotors.SoftwareControlledMotor;
import edu.wpi.first.wpilibj.sensing.VelocitySource;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

public class SoftwareControlledElevator {

    private final SoftwareControlledMotor controlledMotor;
    private final double gravityCompensationTorquePercentage;

    public SoftwareControlledElevator(SpeedController voltageController,
                                      Encoder encoder,
                                      double gravityCompensationTorquePercentage,
                                      double positionUnitsPerSensorUnit,
                                      double sensorValueAtZero,
                                      double maxSpeedInSensorUnitsPerSecond) {

        this(voltageController,
                PositionAndVelocitySource.from(
                        encoder::getDistance,
                        encoder::getRate),
                gravityCompensationTorquePercentage,
                positionUnitsPerSensorUnit,
                sensorValueAtZero,
                maxSpeedInSensorUnitsPerSecond);
    }

    public SoftwareControlledElevator(SpeedController voltageController,
                                      Potentiometer potentiometer,
                                      double gravityCompensationTorquePercentage,
                                      double positionUnitPerSensorUnit,
                                      double sensorValueAtZero,
                                      double maxSpeedInSensorUnitsPerSecond) {

        this(voltageController,
                PositionAndVelocitySource.from(
                        potentiometer::get,
                        VelocitySource.differentiatePosition(potentiometer::get)),
                gravityCompensationTorquePercentage,
                positionUnitPerSensorUnit,
                sensorValueAtZero,
                maxSpeedInSensorUnitsPerSecond);
    }

    public SoftwareControlledElevator(SpeedController voltageController,
                                      PositionAndVelocitySource stateSource,
                                      double gravityCompensationTorquePercentage,
                                      double positionUnitPerSensorUnit,
                                      double sensorValueAtZero,
                                      double maxSpeedInSensorUnitsPerSecond) {

        this.gravityCompensationTorquePercentage =
                gravityCompensationTorquePercentage;

        final PositionAndVelocitySource levelAtZeroDegreesSource =
                PositionAndVelocitySource.corrected(stateSource,
                        -sensorValueAtZero,
                        positionUnitPerSensorUnit);

        this.controlledMotor = SoftwareControlledMotor.builder()
                .withSpeedController(voltageController)
                .withPositionSource(levelAtZeroDegreesSource)
                .withVelocitySource(levelAtZeroDegreesSource)
                .withMaxSpeedInSensorUnitsPerSecond(maxSpeedInSensorUnitsPerSecond)
                .withTorqueOffsetComputer(this::computeGravityCompensationTorquePercent)
                .build();
    }

    public void setPosition(double position) {
        controlledMotor.setDesiredAbsolutePosition(position);
    }

    public double getPosition() {
        return controlledMotor.getPosition();
    }

    private double computeGravityCompensationTorquePercent() {
        return gravityCompensationTorquePercentage;
    }
}