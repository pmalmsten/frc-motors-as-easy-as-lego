package edu.wpi.first.wpilibj.controlledmotors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.sensing.PositionSource;

import java.util.function.Supplier;

public class TalonSRXControlledMotor implements PositionControlledMotor, VelocityControlledMotor {

    private final WPI_TalonSRX talonSRX;

    private final double sensorUnitsOffset;
    private final double positionUnitsPerSensorUnit;
    private final double maxVelocityInSensorUnitsPerSecond;
    private final Supplier<Double> voltagePercentageFeedForwardCalculator;

    private final PIDController positionController;

    public TalonSRXControlledMotor(int channel,
                                   FeedbackDevice feedbackDevice,
                                   Supplier<Double> voltagePercentageFeedForwardCalculator,
                                   double sensorUnitsOffset,
                                   double positionUnitsPerSensorUnit,
                                   double maxVelocityInSensorUnitsPerSecond) {
        talonSRX = new WPI_TalonSRX(channel);
        talonSRX.configSelectedFeedbackSensor(feedbackDevice, 0, 0);
        talonSRX.config_kP(0, 1, 0);

        this.voltagePercentageFeedForwardCalculator = voltagePercentageFeedForwardCalculator;
        this.sensorUnitsOffset = sensorUnitsOffset;
        this.positionUnitsPerSensorUnit = positionUnitsPerSensorUnit;
        this.maxVelocityInSensorUnitsPerSecond = maxVelocityInSensorUnitsPerSecond;

        positionController = new PIDController(1, 0, 0,
                PositionSource.toPIDSource(this), this::updateTalon);
    }

    private void updateTalon(double desiredVelocityPercentage) {
        talonSRX.set(ControlMode.Velocity,
                desiredVelocityPercentage * maxVelocityInSensorUnitsPerSecond);

        double feedForwardVoltagePercentage = voltagePercentageFeedForwardCalculator.get();
        // TODO: Figure out how to send auxiliary feed forward to Talon SRX
    }

    @Override
    public void setDesiredVelocityAsPercentage(double desiredVelocityPercentage) {
        positionController.disable();

        updateTalon(desiredVelocityPercentage);
    }

    @Override
    public void setDesiredAbsolutePosition(double desiredPositionInControllerUnits) {
        if (!positionController.isEnabled()) {
            positionController.reset();
        }

        positionController.setSetpoint(desiredPositionInControllerUnits);
        positionController.enable();
    }

    @Override
    public double getPosition() {
        return (talonSRX.getSelectedSensorPosition(0) + sensorUnitsOffset) * positionUnitsPerSensorUnit;
    }

    @Override
    public double getVelocity() {
        return talonSRX.getSelectedSensorVelocity(0) * positionUnitsPerSensorUnit;
    }
}
