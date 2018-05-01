package edu.wpi.first.wpilibj.controlledmotors;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.sensing.PositionSource;
import edu.wpi.first.wpilibj.sensing.VelocitySource;

import java.util.Objects;
import java.util.function.Supplier;

public class SoftwareControlledMotor implements VelocityControlledMotor, PositionControlledMotor {

    private final PositionSource positionSource;
    private final VelocitySource velocitySource;

    private final double maxSpeedInSensorUnitsPerSecond;

    private final PIDController velocityController;
    private final PIDController positionController;

    SoftwareControlledMotor(Builder builder) {
        if (builder.positionUnitsPerSensorUnit == 0) {
            throw new IllegalArgumentException("positionUnitsPerSensorUnit must not be zero");
        }

        this.positionSource = configurePositionSource(builder);
        this.velocitySource = configureVelocitySource(builder);

        this.maxSpeedInSensorUnitsPerSecond = builder.maxSpeedInSensorUnitsPerSecond;
        if (maxSpeedInSensorUnitsPerSecond <= 0) {
            throw new IllegalArgumentException("maxSpeedInSensorUnitsPerSecond must be" +
                    "greater than zero");
        }

        final PIDOutput torqueController =
                withLinearTorque(builder.voltageController, velocitySource);

        final PIDOutput adjustedTorqueController =
                withTorqueOffset(torqueController, builder.torqueOffsetComputer);

        // Cascaded PID with inner loop running 5x faster
        velocityController = new PIDController(1, 0, 0,
                VelocitySource.toDisplacementPIDSource(velocitySource),
                adjustedTorqueController, 0.010);

        positionController = new PIDController(1, 0, 0,
                PositionSource.toPIDSource(positionSource),
                velocityController::setSetpoint);


    }

    private PIDOutput withTorqueOffset(PIDOutput torqueController, Supplier<Double> torqueAdjuster) {
        Objects.requireNonNull(torqueAdjuster, "torqueOffsetComputer must not be null");

        return (desiredTorquePercentage) ->
                torqueController.pidWrite(desiredTorquePercentage +
                        torqueAdjuster.get());
    }

    private VelocitySource configureVelocitySource(Builder builder) {
        final VelocitySource rawVelocitySource = Objects.requireNonNull(builder.velocitySource,
                "velocitySource must not be null");

        return VelocitySource.scaled(rawVelocitySource, builder.positionUnitsPerSensorUnit);
    }

    private PositionSource configurePositionSource(Builder builder) {
        final PositionSource rawPositionSource = Objects.requireNonNull(builder.positionSource,
                "positionSource must not be null");

        return PositionSource.corrected(rawPositionSource, 0,
                builder.positionUnitsPerSensorUnit);
    }

    private PIDOutput withLinearTorque(SpeedController voltageController,
                                       VelocitySource velocitySource) {

        // Leverage motor curve information to linearize torque output across varying RPM
        // as best we can. We know that max torque is available at 0 RPM and zero torque is
        // available at max RPM - use that relationship to proportionately boost voltage output
        // as motor speed increases.

        return desiredTorquePercentage -> {
            final double currentSpeedSensorUnitsPerSecond =
                    Math.abs(velocitySource.getVelocity());
            final double currentSpeedAsPercentOfMax =
                    currentSpeedSensorUnitsPerSecond / maxSpeedInSensorUnitsPerSecond;

            // Max torque percentage declines proportionally to motor speed.
            final double percentMaxTorqueAvailable = 1 - currentSpeedAsPercentOfMax;

            final double desiredVoltagePercentage;
            if (percentMaxTorqueAvailable > 0) {
                double correctionFactor = 1 / percentMaxTorqueAvailable;
                desiredVoltagePercentage = desiredTorquePercentage * correctionFactor;
            } else {
                // When we exceed max motor speed (and the correction factor is undefined),
                // apply 100% voltage.
                desiredVoltagePercentage = Math.signum(desiredTorquePercentage);
            }

            voltageController.set(desiredVoltagePercentage);
        };
    }

    @Override
    public void setDesiredVelocityAsPercentage(double desiredVelocityPercentage) {
        positionController.disable();

        velocityController.setSetpoint(desiredVelocityPercentage * maxSpeedInSensorUnitsPerSecond);
        velocityController.enable();
    }

    @Override
    public void setDesiredAbsolutePosition(double desiredPositionInControllerUnits) {
        if (!positionController.isEnabled()) {
            positionController.reset();
        }

        positionController.setSetpoint(desiredPositionInControllerUnits);
        positionController.enable();
        velocityController.enable();
    }

    @Override
    public double getPosition() {
        return positionSource.getPosition();
    }

    @Override
    public double getVelocity() {
        return velocitySource.getVelocity();
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private SpeedController voltageController;
        private PositionSource positionSource;
        private VelocitySource velocitySource;
        private double maxSpeedInSensorUnitsPerSecond;

        private double positionUnitsPerSensorUnit = 1.0;
        private Supplier<Double> torqueOffsetComputer = () -> 0.0; // Default to no-op

        public Builder withSpeedController(SpeedController speedController) {
            this.voltageController = speedController;
            return this;
        }

        public Builder withPositionSource(PositionSource positionSource) {
            this.positionSource = positionSource;
            return this;
        }

        public Builder withVelocitySource(VelocitySource velocitySource) {
            this.velocitySource = velocitySource;
            return this;
        }

        public Builder withEncoder(Encoder encoder) {
            return this
                    .withPositionSource(encoder::getDistance)
                    .withVelocitySource(encoder::getRate);
        }


        public Builder withMaxSpeedInSensorUnitsPerSecond(double maxVelocityInSensorUnitsPerSecond) {
            this.maxSpeedInSensorUnitsPerSecond = maxVelocityInSensorUnitsPerSecond;
            return this;
        }

        public Builder positionUnitsPerSensorUnit(double positionUnitsPerSensorUnit) {
            this.positionUnitsPerSensorUnit = positionUnitsPerSensorUnit;
            return this;
        }

        public Builder withTorqueOffsetComputer(Supplier<Double> torqueAdjuster) {
            this.torqueOffsetComputer = torqueAdjuster;
            return this;
        }

        public SoftwareControlledMotor build() {
            return new SoftwareControlledMotor(this);
        }
    }
}
