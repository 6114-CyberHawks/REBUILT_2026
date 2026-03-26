package frc.robot.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Calculates optimal shooter settings based on distance to target.
 * Uses zone-based interpolation for RPM and hood angle.
 */
public class ShooterCalculator {

    /**
     * Container for calculated shooter settings
     */
    public static class ShooterSettings {
        public final double rpm;
        public final double hoodPosition;
        public final double distance;
        public final boolean isValid;

        public ShooterSettings(double rpm, double hoodPosition, double distance, boolean isValid) {
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
            this.distance = distance;
            this.isValid = isValid;
        }

        public static ShooterSettings invalid() {
            return new ShooterSettings(0, 0, 0, false);
        }
    }

    /**
     * Calculate shooter settings based on distance to target using zone interpolation
     * @param distanceInches Distance to target in inches
     * @return ShooterSettings with RPM and hood position
     */
    public static ShooterSettings calculateFromDistance(double distanceInches) {
        double[][] zones = Constants.ShooterConstants.SHOOTING_ZONES;

        // Validate distance
        if (distanceInches <= 0 || Double.isNaN(distanceInches) || Double.isInfinite(distanceInches)) {
            SmartDashboard.putString("Shooter/Calc Status", "Invalid distance");
            return ShooterSettings.invalid();
        }

        // Closer than minimum zone - use first zone values
        if (distanceInches <= zones[0][0]) {
            SmartDashboard.putString("Shooter/Calc Status", "At minimum range");
            return new ShooterSettings(zones[0][1], zones[0][2], distanceInches, true);
        }

        // Farther than maximum zone - use last zone values
        if (distanceInches >= zones[zones.length - 1][0]) {
            SmartDashboard.putString("Shooter/Calc Status", "At maximum range");
            return new ShooterSettings(
                zones[zones.length - 1][1], 
                zones[zones.length - 1][2], 
                distanceInches, 
                true
            );
        }

        // Find the two zones to interpolate between
        for (int i = 0; i < zones.length - 1; i++) {
            double lowerDist = zones[i][0];
            double upperDist = zones[i + 1][0];

            if (distanceInches >= lowerDist && distanceInches <= upperDist) {
                // Linear interpolation
                double ratio = (distanceInches - lowerDist) / (upperDist - lowerDist);

                double rpm = zones[i][1] + (zones[i + 1][1] - zones[i][1]) * ratio;
                double hood = zones[i][2] + (zones[i + 1][2] - zones[i][2]) * ratio;

                SmartDashboard.putString("Shooter/Calc Status", 
                    String.format("Interpolating zones %d-%d", i, i+1));
                SmartDashboard.putNumber("Shooter/Calc Ratio", ratio);

                return new ShooterSettings(rpm, hood, distanceInches, true);
            }
        }

        // Fallback (should never reach here)
        SmartDashboard.putString("Shooter/Calc Status", "Fallback - using default");
        return new ShooterSettings(
            Constants.ShooterConstants.DEFAULT_SHOOTER_RPM, 
            0.5, 
            distanceInches, 
            true
        );
    }

    /**
     * Check if current shooter state matches target settings
     */
    public static boolean isAtTarget(double currentRPM, double targetRPM, 
                                      double currentHood, double targetHood) {
        boolean rpmReady = Math.abs(currentRPM - targetRPM) < Constants.ShooterConstants.RPM_TOLERANCE;
        boolean hoodReady = Math.abs(currentHood - targetHood) < Constants.ShooterConstants.HOOD_TOLERANCE;
        return rpmReady && hoodReady;
    }
}