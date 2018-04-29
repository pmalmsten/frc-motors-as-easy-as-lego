package edu.wpi.first.wpilibj.sensing;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public interface PositionSource {

    /**
     * @return The position of this entity in units defined by the entity.
     */
    double getPosition();

    static PositionSource corrected(PositionSource source,
                                    double offsetInSourceUnits,
                                    double scaleFactor) {
        return () -> (source.getPosition() + offsetInSourceUnits) * scaleFactor;
    }

    static PIDSource toPIDSource(PositionSource source) {
        return new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                throw new UnsupportedOperationException();
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return source.getPosition();
            }
        };
    }
}
