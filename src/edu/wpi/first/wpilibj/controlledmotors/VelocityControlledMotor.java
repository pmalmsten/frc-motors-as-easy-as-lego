package edu.wpi.first.wpilibj.controlledmotors;

import edu.wpi.first.wpilibj.sensing.VelocitySource;

public interface VelocityControlledMotor extends VelocitySource {

    void setDesiredVelocityAsPercentage(double desiredVelocityPercentage);

}
