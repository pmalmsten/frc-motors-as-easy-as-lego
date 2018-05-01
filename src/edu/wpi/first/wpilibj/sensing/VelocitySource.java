package edu.wpi.first.wpilibj.sensing;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotController;

import java.util.concurrent.TimeUnit;

public interface VelocitySource {

    /**
     * @return The velocity of this entity in units defined by the entity.
     */
    double getVelocity();

    static VelocitySource scaled(VelocitySource source,
                                 double scaleFactor) {
        return () -> source.getVelocity() * scaleFactor;
    }

    /**
     * Produces a PIDSource that causes consuming {@link edu.wpi.first.wpilibj.PIDController}s to
     * operate in traditional PID control mode as opposed to the "ID" control mode the controller uses
     * when a {@link PIDSourceType#kRate} is provided.
     *
     * This is useful when the output of the consuming {@link edu.wpi.first.wpilibj.PIDController}
     * will be interpreted as a torque to be sent to a motor instead of voltage.
     *
     * @param source The {@link VelocitySource} to convert to {@link PIDSource}.
     * @return A source which marks given values as {@link PIDSourceType#kDisplacement}.
     */
    static PIDSource toDisplacementPIDSource(VelocitySource source) {
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
                return source.getVelocity();
            }
        };
    }

    static VelocitySource differentiatePosition(PositionSource positionSource) {
        // Thread safe.
        return new VelocitySource() {
            final double MIN_DIFFERENTIATION_PERIOD_MS = 5;

            double cachedVelocity;
            double lastPosition;
            long lastPositionTimestampMs = -1;

            @Override
            public synchronized double getVelocity() {
                final long currentTimeMs = TimeUnit.MICROSECONDS.toMillis(RobotController.getFPGATime());
                if (previousPositionKnown() && cachedVelocityFresh(currentTimeMs)) {
                    // Short-circuit since the last velocity value is fresh
                    return cachedVelocity;
                }

                final double computedVelocity;
                final double currentPosition = positionSource.getPosition();

                if (previousPositionKnown()) {
                    double timeDeltaSeconds = (currentTimeMs - lastPositionTimestampMs) / 1000;
                    double positionDelta = currentPosition - lastPosition;

                    computedVelocity = positionDelta / timeDeltaSeconds;
                } else {
                    // Default to 0 when no previous sample is available.
                    computedVelocity = 0;
                }

                lastPosition = currentPosition;
                lastPositionTimestampMs = currentTimeMs;
                cachedVelocity = computedVelocity;

                return computedVelocity;
            }

            private boolean previousPositionKnown() {
                return lastPositionTimestampMs > 0;
            }

            private boolean cachedVelocityFresh(long currentTimeMs) {
                return currentTimeMs - lastPositionTimestampMs < MIN_DIFFERENTIATION_PERIOD_MS;
            }
        };
    }
}
