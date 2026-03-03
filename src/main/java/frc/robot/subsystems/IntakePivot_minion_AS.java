package frc.robot.subsystems;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.MotorType;
import com.thethriftybot.ThriftyNova.IdleMode;
import com.thethriftybot.ThriftyNova.PIDSlot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

// Simulation imports
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakePivot extends SubsystemBase {
    private final ThriftyNova m_leaderMotor;
    private final ThriftyNova m_followerMotor;
    
    // Position constants (in rotations of OUTPUT shaft)
    private static final double STOWED_POSITION = 0.0;      // Up position
    private static final double DEPLOYED_POSITION = 0.25;   // Down position (90 degrees)
    
    // Soft limits
    private static final double FORWARD_SOFT_LIMIT = 0.26;   // ~93 degrees
    private static final double REVERSE_SOFT_LIMIT = -0.01;  // Slightly before zero
    
    // PID Constants (tunable via dashboard)
    private double m_kP = 2.0;    // Proportional gain
    private double m_kI = 0.0;    // Integral gain
    private double m_kD = 0.1;    // Derivative gain
    
    // Gear ratio: 23:1
    private static final double GEAR_RATIO = 23.0;
    
    // Position tolerance
    private static final double POSITION_TOLERANCE = 0.02;  // ~7 degrees
    
    // Current target position
    private double m_targetPosition = STOWED_POSITION;
    
    // Simulation objects
    private SingleJointedArmSim m_armSim;
    private Mechanism2d m_mech2d;
    private MechanismLigament2d m_armVisual;
    private double m_simPosition = 0.0;
    
    public IntakePivot(int leaderCANId, int followerCANId) {
        m_leaderMotor = new ThriftyNova(leaderCANId, MotorType.Brushless);
        m_followerMotor = new ThriftyNova(followerCANId, MotorType.Brushless);
        
        configureMotors();
        
        // Initialize tunable PID values on dashboard
        SmartDashboard.putNumber("IntakePivot/Tuning/kP", m_kP);
        SmartDashboard.putNumber("IntakePivot/Tuning/kI", m_kI);
        SmartDashboard.putNumber("IntakePivot/Tuning/kD", m_kD);
        
        if (RobotBase.isSimulation()) {
            setupSimulation();
        }
    }
    
    private void configureMotors() {
        // Factory reset to clear any previous settings
        m_leaderMotor.factoryReset();
        m_followerMotor.factoryReset();
        
        // Configure leader motor
        m_leaderMotor.setInverted(false);  // Adjust based on testing
        m_leaderMotor.setIdleMode(IdleMode.Brake);
        
        // Set encoder conversion factor (rotations of OUTPUT shaft)
        m_leaderMotor.setPositionConversionFactor(1.0 / GEAR_RATIO);
        m_leaderMotor.setVelocityConversionFactor(1.0 / GEAR_RATIO);
        
        // Configure PID
        m_leaderMotor.setPID(PIDSlot.Slot0, m_kP, m_kI, m_kD);
        
        // Set output limits
        m_leaderMotor.setOutputRange(PIDSlot.Slot0, -0.5, 0.5);
        
        // Configure soft limits
        m_leaderMotor.enableSoftLimits(true);
        m_leaderMotor.setSoftLimits(REVERSE_SOFT_LIMIT, FORWARD_SOFT_LIMIT);
        
        // Set current limit (Minion motors can handle ~40A)
        m_leaderMotor.setCurrentLimit(40);
        
        // Configure follower motor
        m_followerMotor.follow(m_leaderMotor, false);  // false = same direction
        m_followerMotor.setIdleMode(IdleMode.Brake);
        m_followerMotor.setCurrentLimit(40);
        
        // Zero encoder at startup - IMPORTANT: Arm must be stowed (up)!
        m_leaderMotor.setEncoderPosition(0.0);
        
        // Burn settings to flash
        m_leaderMotor.burnFlash();
        m_followerMotor.burnFlash();
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
        SmartDashboard.putNumber("IntakePivot/Velocity (rot/s)", 
            m_leaderMotor.getEncoderVelocity());
        SmartDashboard.putNumber("IntakePivot/Velocity (deg/s)", 
            m_leaderMotor.getEncoderVelocity() * 360.0);
        
        // ===== MOTOR HEALTH DATA =====
        SmartDashboard.putNumber("IntakePivot/Leader Current (A)", 
            m_leaderMotor.getOutputCurrent());
        SmartDashboard.putNumber("IntakePivot/Follower Current (A)", 
            m_followerMotor.getOutputCurrent());
        SmartDashboard.putNumber("IntakePivot/Total Current (A)", 
            m_leaderMotor.getOutputCurrent() + m_followerMotor.getOutputCurrent());
        SmartDashboard.putNumber("IntakePivot/Applied Output", 
            m_leaderMotor.getAppliedOutput());
        SmartDashboard.putNumber("IntakePivot/Bus Voltage", 
            m_leaderMotor.getBusVoltage());
        SmartDashboard.putNumber("IntakePivot/Leader Temperature (C)", 
            m_leaderMotor.getMotorTemperature());
        SmartDashboard.putNumber("IntakePivot/Follower Temperature (C)", 
            m_followerMotor.getMotorTemperature());
        
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
            m_leaderMotor.setPID(PIDSlot.Slot0, m_kP, m_kI, m_kD);
            System.out.println("PID UPDATED: kP=" + m_kP + " kI=" + m_kI + " kD=" + m_kD);
        }
        
        // ===== STATUS DATA =====
        SmartDashboard.putBoolean("IntakePivot/At Target", atTarget());
        SmartDashboard.putString("IntakePivot/State", 
            atTarget() ? "AT_TARGET" : "MOVING");
        SmartDashboard.putBoolean("IntakePivot/At Forward Limit", 
            getPosition() >= FORWARD_SOFT_LIMIT - 0.01);
        SmartDashboard.putBoolean("IntakePivot/At Reverse Limit", 
            getPosition() <= REVERSE_SOFT_LIMIT + 0.01);
        
        // ===== DIAGNOSTIC DATA =====
        SmartDashboard.putBoolean("IntakePivot/Diagnostics/Leader Fault", 
            m_leaderMotor.getFaults() != 0);
        SmartDashboard.putBoolean("IntakePivot/Diagnostics/Follower Fault", 
            m_followerMotor.getFaults() != 0);
    }
    
    @Override
    public void simulationPeriodic() {
        // Simple simulation without Thrifty-specific sim support
        double voltage = m_leaderMotor.getAppliedOutput() * 12.0;
        
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
    }
    
    // ===== COMMAND METHODS =====
    
    public void deployIntake() {
        m_targetPosition = DEPLOYED_POSITION;
        m_leaderMotor.setReference(m_targetPosition, ThriftyNova.ControlType.Position, PIDSlot.Slot0);
        System.out.println("DEPLOY commanded - target: " + m_targetPosition + " rotations (" 
            + (m_targetPosition * 360.0) + " degrees)");
    }
    
    public void stowIntake() {
        m_targetPosition = STOWED_POSITION;
        m_leaderMotor.setReference(m_targetPosition, ThriftyNova.ControlType.Position, PIDSlot.Slot0);
        System.out.println("STOW commanded - target: " + m_targetPosition + " rotations (" 
            + (m_targetPosition * 360.0) + " degrees)");
    }
    
    public void stop() {
        // Hold current position with PID
        m_targetPosition = getPosition();
        m_leaderMotor.setReference(m_targetPosition, ThriftyNova.ControlType.Position, PIDSlot.Slot0);
        System.out.println("STOP - holding position: " + m_targetPosition + " rotations (" 
            + (m_targetPosition * 360.0) + " degrees)");
    }
    
    // ===== QUERY METHODS =====
    
    public boolean atTarget() {
        return Math.abs(m_targetPosition - getPosition()) < POSITION_TOLERANCE;
    }
    
    public double getPosition() {
        if (RobotBase.isSimulation()) {
            return m_simPosition;
        }
        return m_leaderMotor.getEncoderPosition();
    }
    
    public double getPositionDegrees() {
        return getPosition() * 360.0;
    }
    
    public double getVelocity() {
        return m_leaderMotor.getEncoderVelocity();
    }
    
    public double getVelocityDegreesPerSecond() {
        return getVelocity() * 360.0;
    }
    
    public double getTargetPosition() {
        return m_targetPosition;
    }
    
    public double getPositionError() {
        return m_targetPosition - getPosition();
    }
}
