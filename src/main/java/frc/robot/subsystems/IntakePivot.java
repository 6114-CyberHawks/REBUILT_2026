
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

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
    private static final double DEPLOY_SPEED = 0.3;
    private static final double STOW_SPEED = 0.5;
    
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
        leaderConfig.inverted(true);
        
        m_leaderMotor.configure(
            leaderConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(m_leaderMotor);
        followerConfig.smartCurrentLimit(40);
        followerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        
        m_followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters,
                                 PersistMode.kPersistParameters);
        
        m_leaderMotor.getEncoder().setPosition(0.0);
    }
    
    private void setupSimulation() {
        m_leaderSim = new SparkSim(m_leaderMotor, DCMotor.getNEO(2));
        // Arm physical parameters
        double armMassKg = 2.04263;              // Estimated arm mass
        double armLengthMeters = 0.310;        // Estimated arm length
        
        // Calculate moment of inertia (MOI) for rod rotating about one end
        double moiKgMetersSquared = armMassKg * Math.pow(armLengthMeters, 2) / 3.0;
        
        // Create arm simulator with correct constructor【toolu_vrtx_011emFoS42wdZ9LX53oNMprg-2】
        m_armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),                  // Gearbox: 2 NEO motors
            23.0,                               // Gear ratio: 23:1
            moiKgMetersSquared,                 // Moment of inertia (kg*m^2)
            armLengthMeters,                    // Arm length (meters)
            Units.degreesToRadians(0),          // Min angle (radians)
            Units.degreesToRadians(93),         // Max angle (radians)
            true,                               // Simulate gravity
            Units.degreesToRadians(0)           // Starting angle (radians)
        );
        
        // Create 2D mechanism visualization
        m_mech2d = new Mechanism2d(2, 2);
        MechanismRoot2d root = m_mech2d.getRoot("IntakePivot", 1, 0.5);
        m_armVisual = root.append(
            new MechanismLigament2d(
                "Intake Arm",
                armLengthMeters,
                90,                             // Starting angle (degrees)
                6,                              // Line width
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
        // Get motor voltage
        double voltage = m_leaderSim.getAppliedOutput() * 12.0;
        
        // Update simulation
        m_armSim.setInputVoltage(voltage);
        m_armSim.update(0.02);  // 20ms timestep
        
        //Get simulated velocity
        double velocityRadPerSec = m_armSim.getVelocityRadPerSec();

        //Update SparkMax sim with velocity
        m_leaderSim.iterate(velocityRadPerSec, 12.0, 0.02);
     
        
        // Get simulated position
        double angleRadians = m_armSim.getAngleRads();
        m_simPosition = angleRadians / (2 * Math.PI);
        
        // Update visual (90 degree offset for display)
        m_armVisual.setAngle(90 - Units.radiansToDegrees(angleRadians));
        
        // Publish simulation data
        SmartDashboard.putNumber("IntakePivot/Sim Angle (deg)", 
            Units.radiansToDegrees(angleRadians));
        SmartDashboard.putNumber("IntakePivot/Sim Current (A)", 
            m_armSim.getCurrentDrawAmps());
    }
    
    public void deployIntake() {
        m_leaderMotor.set(DEPLOY_SPEED);
    }
    
    public void stowIntake() {
        m_leaderMotor.set(-STOW_SPEED);
    }
    
    public void stop() {
        m_leaderMotor.set(0.0);
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

