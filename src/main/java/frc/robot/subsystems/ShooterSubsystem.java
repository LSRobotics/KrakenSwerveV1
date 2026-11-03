package frc.robot.subsystems;
 
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_leftMotor =
      new SparkMax(ShooterConstants.kLeftMotorId, MotorType.kBrushless);
  private final SparkMax m_rightMotor =
      new SparkMax(ShooterConstants.kRightMotorId, MotorType.kBrushless);
  private final SparkMax m_indexMotor =
      new SparkMax(ShooterConstants.kIndexMotorId, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
   
  public ShooterSubsystem() {
    // Left motor: the "leader"
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kCurrentLimitAmps)
        .inverted(true);
 
    // Right motor: mirrored on the robot, so it follows the left and is inverted
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kCurrentLimitAmps)
        .follow(m_leftMotor, true); // true = opposite direction of the leader

    // Index motor: Don't know if it is inveresed on not.  We shall see when we test
    SparkMaxConfig indexConfig = new SparkMaxConfig();
    indexConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kCurrentLimitAmps)
        .inverted(true); 
 
    m_leftMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_indexMotor.configure(
        indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
 

    /** Sets shooter output as a percentage (-1.0 to 1.0). */
  public void setSpeed(double speed) {
    m_leftMotor.set(speed); // right motor follows (inverted)
  }

  /** Sets shooter output as a percentage (-1.0 to 1.0). */
  public void setIndexSpeed(double speed) {
    m_indexMotor.set(speed); // 
  }
 
  public void stopShooter() {
    m_leftMotor.stopMotor();
  }

  public void stopIndexer() {
    m_indexMotor.stopMotor();
  }
 
    /*--------------------------------------------------------------------- */
    /*  Getting the RPM from the encoders so we can put it on the dashboard */
    //
    public double getShooterRPM() {
      return m_leftEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
    }

    /*---------------------------------------------------------------------- */



  /** Spins the shooter while the command is scheduled, stops when it ends. */
  public Command spinCommand() {
    return runEnd(() -> setSpeed(ShooterConstants.kShooterSpeed), this::stopShooter)
        .withName("ShooterSpin");
  }

    /** Spins the shooter while the command is scheduled, stops when it ends. */
  public Command indexCommand() {
    return runEnd(() -> setIndexSpeed(ShooterConstants.kIndexSpeed), this::stopIndexer)
        .withName("IndexSpin");
  }
}