package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
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
    private final SparkMax m_leaderMotor;
    private final SparkMax m_followerMotor;
    
    // Soft limits
    private static final double FORWARD_SOFT_LIMIT = 0.26;
    private static final double REVERSE_SOFT_LIMIT = -0.01;
    
    // Motor speeds
    private static final double DEPLOY_SPEED = 0.1;
    private static final double STOW_SPEED = 0.1;
    
    // Holding voltage to fight gravity when stopped
    private static final double HOLD_VOLTAGE = -0.08;  // Tune this value (start small!)
    
    // Simulation objects
    private SparkSim m_leaderSim;
    private SingleJointedArmSim m_armSim;
    private Mechanism2d m_mech2d;
    private MechanismLigament2d m_armVisual;
    private double m_simPosition = 0.0;
    
    public IntakePivot(int leaderCANId, int followerCANId) {
        m_leaderMotor = new SparkMax(leaderCANId, MotorType.kBrushless);
        m_followerMotor = new SparkMax(followerCANId, MotorType.kBrushless);
        
        configureMotors();
        
        if (RobotBase.isSimulation()) {
            setupSimulation();
        }
    }
    
    private void configureMotors() {
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        
        leaderConfig.encoder
            .positionConversionFactor(1.0 / 23.0)
            .velocityConversionFactor(1.0 / 23.0);
        
        leaderConfig.softLimit
            .forwardSoftLimit(FORWARD_SOFT_LIMIT)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(REVERSE_SOFT_LIMIT)
            .reverseSoftLimitEnabled(true);
        
        leaderConfig.smartCurrentLimit(40);
        leaderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leaderConfig.inverted(true);  // Adjust based on your testing
        
        m_leaderMotor.configure(
            leaderConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
        
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(m_leaderMotor);
        followerConfig.smartCurrentLimit(40);
        followerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        
        m_followerMotor.configure(
            followerConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
        
        // Zero encoder at startup - IMPORTANT: Arm must be stowed (up) when robot starts!
        m_leaderMotor.getEncoder().setPosition(0.0);
    }
    
    private void setupSimulation() {
        m_leaderSim = new SparkSim(m_leaderMotor, DCMotor.getNEO(2));
        
        double armMassKg = 2.0;
        double armLengthMeters = 0.5;
        double moiKgMetersSquared = armMassKg * Math.pow(armLengthMeters, 2) / 3.0;
        
        m_armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            23.0,
            moiKgMetersSquared,
            armLengthMeters,
            Units.degreesToRadians(0),
            Units.degreesToRadians(93),
            true,
            Units.degreesToRadians(0)
        );
        
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
        SmartDashboard.putNumber("IntakePivot/Position (rotations)", getPosition());
        SmartDashboard.putNumber("IntakePivot/Position (degrees)", getPositionDegrees());
        SmartDashboard.putNumber("IntakePivot/Leader Current (A)", 
            m_leaderMotor.getOutputCurrent());
        SmartDashboard.putNumber("IntakePivot/Applied Output", 
            m_leaderMotor.getAppliedOutput());
        SmartDashboard.putBoolean("IntakePivot/At Forward Limit", 
            getPosition() >= FORWARD_SOFT_LIMIT - 0.01);
        SmartDashboard.putBoolean("IntakePivot/At Reverse Limit", 
            getPosition() <= REVERSE_SOFT_LIMIT + 0.01);
    }
    
    @Override
    public void simulationPeriodic() {
        double voltage = m_leaderSim.getAppliedOutput() * 12.0;
        
        m_armSim.setInputVoltage(voltage);
        m_armSim.update(0.02);
        
        double velocityRadPerSec = m_armSim.getVelocityRadPerSec();
        m_leaderSim.iterate(velocityRadPerSec, 12.0, 0.02);
        
        double angleRadians = m_armSim.getAngleRads();
        m_simPosition = angleRadians / (2 * Math.PI);
        
        m_armVisual.setAngle(90 - Units.radiansToDegrees(angleRadians));
        
        SmartDashboard.putNumber("IntakePivot/Sim Angle (deg)", 
            Units.radiansToDegrees(angleRadians));
        SmartDashboard.putNumber("IntakePivot/Sim Current (A)", 
            m_armSim.getCurrentDrawAmps());
    }
    
    public void deployIntake() {
        System.out.println("DEPLOY CALLED - Setting speed to " + DEPLOY_SPEED);
        m_leaderMotor.set(DEPLOY_SPEED);
    }
    
    public void stowIntake() {
        System.out.println("STOW CALLED - Setting speed to " + (-STOW_SPEED));
        m_leaderMotor.set(-STOW_SPEED);
    }
    
    public void stop() {
        System.out.println("STOP CALLED - Applying hold voltage: " + HOLD_VOLTAGE);
        // Apply small upward voltage to hold against gravity
        m_leaderMotor.setVoltage(HOLD_VOLTAGE);
    }
    
    public double getPosition() {
        if (RobotBase.isSimulation()) {
            return m_simPosition;
        }
        return m_leaderMotor.getEncoder().getPosition();
    }
    
    public double getPositionDegrees() {
        return getPosition() * 360.0;
    }
}
