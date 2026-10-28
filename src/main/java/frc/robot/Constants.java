package frc.robot;
 
public final class Constants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
 
  public static final class ShooterConstants {
    public static final int kLeftMotorId = 24;
    public static final int kRightMotorId = 25;
 
    // Open-loop percent output (-1.0 to 1.0). Tune this for your shooter.
    public static final double kShooterSpeed = 0.8;
 
    public static final int kCurrentLimitAmps = 40;
  }
}