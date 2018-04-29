package edu.wpi.first.wpilibj.sensing;

public interface PositionAndVelocitySource extends PositionSource, VelocitySource {

    static PositionAndVelocitySource from(PositionSource positionSource,
                                          VelocitySource velocitySource) {
        return new PositionAndVelocitySource() {
            @Override
            public double getPosition() {
                return positionSource.getPosition();
            }

            @Override
            public double getVelocity() {
                return velocitySource.getVelocity();
            }
        };
    }

    static PositionAndVelocitySource corrected(PositionAndVelocitySource source,
                                               double sensorUnitsPositionOffset,
                                               double scaleFactor) {

        return new PositionAndVelocitySource() {
            @Override
            public double getPosition() {
                return (source.getPosition() + sensorUnitsPositionOffset) * scaleFactor;
            }

            @Override
            public double getVelocity() {
                return source.getVelocity() * scaleFactor;
            }
        };
    }
}
