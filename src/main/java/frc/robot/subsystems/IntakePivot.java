//Minion motors with Nova controllers.  No PID or AdvatageScope

package frc.robot.subsystems;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
    private final ThriftyNova m_leaderMotor;
    private final ThriftyNova m_followerMotor;
    
    // Soft limits (in motor rotations - internal encoder counts)
    // Gear ratio: 23:1 (23 motor rotations = 1 output rotation)
    private static final double GEAR_RATIO = 23.0;
    
    // Output positions (what we think in terms of)
    private static final double STOWED_OUTPUT_POSITION = 0.0;      // 0 degrees (up)
    private static final double DEPLOYED_OUTPUT_POSITION = 0.25;   // 90 degrees (down)
    
    // Motor limits (multiply by gear ratio for motor encoder counts)
    private static final double REVERSE_SOFT_LIMIT = -0.01 * GEAR_RATIO;  // Slightly before zero
    private static final double FORWARD_SOFT_LIMIT = 0.26 * GEAR_RATIO;   // ~93 degrees (slightly past 90)
    
    // Simple speed constants
    private static final double DEPLOY_SPEED = 0.3;   // 30% power down
    private static final double STOW_SPEED = -0.3;    // 30% power up
    private static final double STOP_SPEED = 0.0;     // Stop
    
    public IntakePivot(int leaderCANId, int followerCANId) {
        m_leaderMotor = new ThriftyNova(leaderCANId, MotorType.Minion);
        m_followerMotor = new ThriftyNova(followerCANId, MotorType.Minion);
        
        configureMotors();
    }
    
    private void configureMotors() {
        // Factory reset
        m_leaderMotor.factoryReset();
        m_followerMotor.factoryReset();
        
        // Basic configuration
        m_leaderMotor.setInverted(false);  // Adjust if needed
        m_leaderMotor.setBrakeMode(true);   // Brake mode for safety
        
        // Configure soft limits (IMPORTANT: These protect your mechanism!)
        m_leaderMotor.setSoftLimits(REVERSE_SOFT_LIMIT, FORWARD_SOFT_LIMIT);
        m_leaderMotor.enableSoftLimits(true);
        
        // Set current limit to protect motors
        m_leaderMotor.setMaxCurrent(CurrentType.STATOR, 40.0);
        
        // Configure follower
        m_followerMotor.follow(m_leaderMotor.getID());
        m_followerMotor.setBrakeMode(true);
        m_followerMotor.setMaxCurrent(CurrentType.STATOR, 40.0);
        
        // Zero encoder at startup - CRITICAL: Arm MUST be in stowed (up) position!
        m_leaderMotor.setEncoderPosition(0.0);
        
        System.out.println("IntakePivot configured:");
        System.out.println("  Soft limits: " + REVERSE_SOFT_LIMIT + " to " + FORWARD_SOFT_LIMIT + " motor rotations");
        System.out.println("  Output range: " + STOWED_OUTPUT_POSITION + " to " + DEPLOYED_OUTPUT_POSITION + " output rotations");
        System.out.println("  Gear ratio: " + GEAR_RATIO + ":1");
        
        // Start stopped
        stop();
    }
    
    // ===== COMMAND METHODS =====
    
    /**
     * Deploy the intake (move down)
     * Soft limits will automatically stop at FORWARD_SOFT_LIMIT
     */
    public void deploy() {
        m_leaderMotor.setPercent(DEPLOY_SPEED);
        System.out.println("DEPLOY - Running at " + (DEPLOY_SPEED * 100) + "% power (soft limit will stop at " + FORWARD_SOFT_LIMIT + ")");
    }
    
    /**
     * Stow the intake (move up)
     * Soft limits will automatically stop at REVERSE_SOFT_LIMIT
     */
    public void stow() {
        m_leaderMotor.setPercent(STOW_SPEED);
        System.out.println("STOW - Running at " + (STOW_SPEED * 100) + "% power (soft limit will stop at " + REVERSE_SOFT_LIMIT + ")");
    }
    
    /**
     * Stop all motion
     */
    public void stop() {
        m_leaderMotor.setPercent(STOP_SPEED);
        System.out.println("STOP - Motors stopped at position: " + m_leaderMotor.getPosition() + " motor rotations");
    }
    
    /**
     * Manual control - set power directly
     * Soft limits still apply!
     * @param speed Speed from -1.0 (full reverse) to 1.0 (full forward)
     */
    public void setSpeed(double speed) {
        m_leaderMotor.setPercent(speed);
    }
    
    /**
     * Get current position in motor rotations
     */
    public double getPositionMotorRotations() {
        return m_leaderMotor.getPosition();
    }
    
    /**
     * Get current position in output rotations (divide by gear ratio)
     */
    public double getPositionOutputRotations() {
        return m_leaderMotor.getPosition() / GEAR_RATIO;
    }
    
    /**
     * Get current position in degrees
     */
    public double getPositionDegrees() {
        return getPositionOutputRotations() * 360.0;
    }
    
    /**
     * Check if at forward soft limit
     */
    public boolean atForwardLimit() {
        return m_leaderMotor.getPosition() >= (FORWARD_SOFT_LIMIT - 0.1);
    }
    
    /**
     * Check if at reverse soft limit
     */
    public boolean atReverseLimit() {
        return m_leaderMotor.getPosition() <= (REVERSE_SOFT_LIMIT + 0.1);
    }
    
    @Override
    public void periodic() {
        // Optional: Print position for debugging (can comment out after testing)
        // System.out.println("Position: " + getPositionDegrees() + " degrees");
    }
}
