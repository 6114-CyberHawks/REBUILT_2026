//Minion motors with Nova controllers.  No PID or AdvatageScope

package frc.robot.subsystems;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
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
<<<<<<< HEAD
        // Use MotorType.Minion for CTR Electronics Minion motors
        m_leaderMotor = new ThriftyNova(leaderCANId, MotorType.MINION);
        m_followerMotor = new ThriftyNova(followerCANId, MotorType.MINION);
=======
        m_leaderMotor = new ThriftyNova(leaderCANId, MotorType.Minion);
        m_followerMotor = new ThriftyNova(followerCANId, MotorType.Minion);
>>>>>>> 67e242184610e8bf59c90c3c84758f2968a0d4c0
        
        configureMotors();
    }
    
    private void configureMotors() {
        // Factory reset
        m_leaderMotor.factoryReset();
        m_followerMotor.factoryReset();
        
        // Basic configuration
        m_leaderMotor.setInverted(false);  // Adjust if needed
        m_leaderMotor.setBrakeMode(true);   // Brake mode for safety
        
<<<<<<< HEAD
        // Configure PID (works with motor rotations, not output rotations)
        m_leaderMotor.pid0.setP(m_kP);
        m_leaderMotor.pid0.setI(m_kI);
        m_leaderMotor.pid0.setD(m_kD);
       
        
        // Set output limits
        m_leaderMotor.setMaxOutput(-0.5, 0.5);
        
        // Configure soft limits (in motor rotations)
        m_leaderMotor.enableSoftLimits(true);
=======
        // Configure soft limits (IMPORTANT: These protect your mechanism!)
>>>>>>> 67e242184610e8bf59c90c3c84758f2968a0d4c0
        m_leaderMotor.setSoftLimits(REVERSE_SOFT_LIMIT, FORWARD_SOFT_LIMIT);
        m_leaderMotor.enableSoftLimits(true);
        
<<<<<<< HEAD
        // Set current limit (Minion motors can handle ~40A)
        m_leaderMotor.setMaxCurrent(CurrentType.STATOR,40.0);
        
        // Configure follower motor
        m_followerMotor.follow(m_leaderMotor.getID());
        m_followerMotor.setBrakeMode(true);  // Brake mode
        m_followerMotor.setMaxCurrent(CurrentType.STATOR,40.0);
        // If follower needs to spin in opposite direction, invert it
        m_followerMotor.setInverted(true);

        // Zero encoder at startup - IMPORTANT: Arm must be stowed (up)!
        m_leaderMotor.setEncoderPosition(0.0);
        

    }
    
    private void setupSimulation() {
        // Minion motor specs (approximate - similar to NEO)
        double armMassKg = 2.0;
        double armLengthMeters = 0.5;
        double moiKgMetersSquared = armMassKg * Math.pow(armLengthMeters, 2) / 3.0;
        
        // Use NEO model as approximation for Minion
        m_armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),  // 2 motors
            GEAR_RATIO,
            moiKgMetersSquared,
            armLengthMeters,
            Units.degreesToRadians(0),
            Units.degreesToRadians(93),
            true,
            Units.degreesToRadians(0)
        );
        
        // Create visualization
        m_mech2d = new Mechanism2d(2, 2);
        MechanismRoot2d root = m_mech2d.getRoot("IntakePivot", 1, 0.5);
        m_armVisual = root.append(
            new MechanismLigament2d(
                "Intake Arm",
                armLengthMeters,
                90,
                6,
                new Color8Bit(Color.kBlue)
            )
        );
        
        SmartDashboard.putData("IntakePivot Mechanism", m_mech2d);
    }
    
    @Override
    public void periodic() {
        // ===== POSITION DATA =====
        SmartDashboard.putNumber("IntakePivot/Position (rotations)", getPosition());
        SmartDashboard.putNumber("IntakePivot/Position (degrees)", getPositionDegrees());
        SmartDashboard.putNumber("IntakePivot/Target Position (rotations)", m_targetPosition);
        SmartDashboard.putNumber("IntakePivot/Target Position (degrees)", 
            m_targetPosition * 360.0);
        SmartDashboard.putNumber("IntakePivot/Position Error (rotations)", 
            m_targetPosition - getPosition());
        SmartDashboard.putNumber("IntakePivot/Position Error (degrees)", 
            (m_targetPosition - getPosition()) * 360.0);
        
        // ===== VELOCITY DATA =====
        SmartDashboard.putNumber("IntakePivot/Velocity (rot/s)", getVelocity());
        SmartDashboard.putNumber("IntakePivot/Velocity (deg/s)", 
            getVelocityDegreesPerSecond());
        
        // ===== MOTOR HEALTH DATA =====
        SmartDashboard.putNumber("IntakePivot/Leader Current (A)", 
            m_leaderMotor.getMaxCurrent());
        SmartDashboard.putNumber("IntakePivot/Follower Current (A)", 
            m_followerMotor.getMaxCurrent());
        SmartDashboard.putNumber("IntakePivot/Total Current (A)", 
            m_leaderMotor.getMaxCurrent() + m_followerMotor.getMaxCurrent());
        SmartDashboard.putNumber("IntakePivot/Applied Output", 
            m_leaderMotor.get());
        SmartDashboard.putNumber("IntakePivot/Bus Voltage", 
            m_leaderMotor.getVoltage());
        SmartDashboard.putNumber("IntakePivot/Leader Temperature (C)", 
            m_leaderMotor.getTemperature());
        SmartDashboard.putNumber("IntakePivot/Follower Temperature (C)", 
            m_followerMotor.getTemperature());
        
        // ===== PID TUNING DATA =====
        SmartDashboard.putNumber("IntakePivot/PID/Current kP", m_kP);
        SmartDashboard.putNumber("IntakePivot/PID/Current kI", m_kI);
        SmartDashboard.putNumber("IntakePivot/PID/Current kD", m_kD);
        
        // Check if PID values changed on dashboard (for live tuning)
        double newKP = SmartDashboard.getNumber("IntakePivot/Tuning/kP", m_kP);
        double newKI = SmartDashboard.getNumber("IntakePivot/Tuning/kI", m_kI);
        double newKD = SmartDashboard.getNumber("IntakePivot/Tuning/kD", m_kD);
        
        if (newKP != m_kP || newKI != m_kI || newKD != m_kD) {
            m_kP = newKP;
            m_kI = newKI;
            m_kD = newKD;
            m_leaderMotor.pid0.setP(m_kP);
            m_leaderMotor.pid0.setI(m_kI);
            m_leaderMotor.pid0.setD(m_kD);
            System.out.println("PID UPDATED: kP=" + m_kP + " kI=" + m_kI + " kD=" + m_kD);
        }
        
        // ===== STATUS DATA =====
        SmartDashboard.putBoolean("IntakePivot/At Target", atTarget());
        SmartDashboard.putString("IntakePivot/State", 
            atTarget() ? "AT_TARGET" : "MOVING");
        SmartDashboard.putBoolean("IntakePivot/At Forward Limit", 
            getPosition() >= (FORWARD_SOFT_LIMIT / GEAR_RATIO) - 0.01);
        SmartDashboard.putBoolean("IntakePivot/At Reverse Limit", 
            getPosition() <= (REVERSE_SOFT_LIMIT / GEAR_RATIO) + 0.01);
        
        // ===== DIAGNOSTIC DATA =====
        SmartDashboard.putBoolean("IntakePivot/Diagnostics/Leader Fault", 
            m_leaderMotor.getErrors().isEmpty());
        SmartDashboard.putBoolean("IntakePivot/Diagnostics/Follower Fault", 
            m_followerMotor.getErrors().isEmpty());
    }
    
    @Override
    public void simulationPeriodic() {
        // Simple simulation without Thrifty-specific sim support
        double voltage = m_leaderMotor.get() * 12.0;
        
        m_armSim.setInputVoltage(voltage);
        m_armSim.update(0.02);
        
        double angleRadians = m_armSim.getAngleRads();
        m_simPosition = angleRadians / (2 * Math.PI);
        
        m_armVisual.setAngle(90 - Units.radiansToDegrees(angleRadians));
        
        // Simulation-specific logging
        SmartDashboard.putNumber("IntakePivot/Sim/Angle (deg)", 
            Units.radiansToDegrees(angleRadians));
        SmartDashboard.putNumber("IntakePivot/Sim/Current (A)", 
            m_armSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("IntakePivot/Sim/Velocity (rad/s)", 
            m_armSim.getVelocityRadPerSec());
=======
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
>>>>>>> 67e242184610e8bf59c90c3c84758f2968a0d4c0
    }
    
    // ===== COMMAND METHODS =====
    
<<<<<<< HEAD
    public void deployIntake() {
        m_targetPosition = DEPLOYED_POSITION;
        m_leaderMotor.setPosition(DEPLOYED_POSITION * GEAR_RATIO);
        System.out.println("DEPLOY commanded - target: " + m_targetPosition + " rotations (" 
            + (m_targetPosition * 360.0) + " degrees)");
    }
    
    public void stowIntake() {
        m_targetPosition = STOWED_POSITION;
        m_leaderMotor.setPosition(STOWED_POSITION * GEAR_RATIO);
        System.out.println("STOW commanded - target: " + m_targetPosition + " rotations (" 
            + (m_targetPosition * 360.0) + " degrees)");
=======
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
>>>>>>> 67e242184610e8bf59c90c3c84758f2968a0d4c0
    }
    
    /**
     * Stop all motion
     */
    public void stop() {
<<<<<<< HEAD
        // Hold current position with PID
        m_targetPosition = getPosition();
        m_leaderMotor.setPosition(m_targetPosition * GEAR_RATIO);
        System.out.println("STOP - holding position: " + m_targetPosition + " rotations (" 
            + (m_targetPosition * 360.0) + " degrees)");
=======
        m_leaderMotor.setPercent(STOP_SPEED);
        System.out.println("STOP - Motors stopped at position: " + m_leaderMotor.getPosition() + " motor rotations");
>>>>>>> 67e242184610e8bf59c90c3c84758f2968a0d4c0
    }
    
    /**
     * Manual control - set power directly
     * Soft limits still apply!
     * @param speed Speed from -1.0 (full reverse) to 1.0 (full forward)
     */
    public void setSpeed(double speed) {
        m_leaderMotor.setPercent(speed);
    }
    
<<<<<<< HEAD
    public double getPosition() {
        if (RobotBase.isSimulation()) {
            return m_simPosition;
        }
        // Convert motor rotations to output rotations
        return m_leaderMotor.getPosition() / GEAR_RATIO;
=======
    /**
     * Get current position in motor rotations
     */
    public double getPositionMotorRotations() {
        return m_leaderMotor.getPosition();
>>>>>>> 67e242184610e8bf59c90c3c84758f2968a0d4c0
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
    
<<<<<<< HEAD
    public double getVelocity() {
        // Convert motor velocity to output velocity
        return m_leaderMotor.getVelocity() / GEAR_RATIO;
=======
    /**
     * Check if at forward soft limit
     */
    public boolean atForwardLimit() {
        return m_leaderMotor.getPosition() >= (FORWARD_SOFT_LIMIT - 0.1);
>>>>>>> 67e242184610e8bf59c90c3c84758f2968a0d4c0
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
