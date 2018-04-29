package edu.wpi.first.wpilibj.controlledmotors;

import edu.wpi.first.wpilibj.sensing.PositionSource;

public interface PositionControlledMotor extends PositionSource {

    void setDesiredAbsolutePosition(double desiredPositionInControllerUnits);

}
