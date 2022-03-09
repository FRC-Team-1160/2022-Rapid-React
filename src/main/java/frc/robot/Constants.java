package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Units are m kg s unless otherwise specified

  // IMPORTANT
  public static final boolean isFinal = true;

  public static final class PortConstants {
    // CAN ID 
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int MIDDLE_LEFT_DRIVE = 2;
    public static final int BACK_LEFT_DRIVE = 3;

    public static final int FRONT_RIGHT_DRIVE = 4;
    public static final int MIDDLE_RIGHT_DRIVE = 5;
    public static final int BACK_RIGHT_DRIVE = 6;

    public static final int INTAKE = 14;
    public static final int MIDDLE_INDEXER = 9;
    public static final int FINAL_INDEXER = 11;

    public static final int LEFT_CLIMBER = 7;
    public static final int RIGHT_CLIMBER = 8;

    public static final int LEFT_SHOOTER = 13;
    public static final int RIGHT_SHOOTER = 10;

    public static final int TURRET = 12;
  }

  public static final class PortConstantsFinal {
    // CAN ID 
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int MIDDLE_LEFT_DRIVE = 2;
    public static final int BACK_LEFT_DRIVE = 3;

    public static final int FRONT_RIGHT_DRIVE = 4;
    public static final int MIDDLE_RIGHT_DRIVE = 5;
    public static final int BACK_RIGHT_DRIVE = 6;

    public static final int INTAKE = 14;
    public static final int MIDDLE_INDEXER = 9;
    public static final int FINAL_INDEXER = 11;

    public static final int LEFT_CLIMBER = 7;
    public static final int RIGHT_CLIMBER = 8;

    public static final int LEFT_SHOOTER = 13;
    public static final int RIGHT_SHOOTER = 10;

    public static final int TURRET = 12;
  }

  public static final class ShooterConstants {
    public static int slotIDx = 0;
    public static double kP = 0;    
    public static double kI = 0;    
    public static double kD = 0;    
    public static int kTimeoutMs = 30;
    public static double HUB_DISTANCE = 16.5; 
    public static double BALL_MASS = 0.27; //kg
    public static double SHOOTER_MASS = 1.0; //kg
    public static double Y_OFFSET = 0.6; //m
    public static double WHEEL_RADIUS = 0.0762; //m
    public static double SHOOTER_ANGLE = 60.0; //radians
    public static double SHOOTER_HEIGHT = 0.66; //m
    public static double LIMELIGHT_ANGLE = 39; //radians
    public static double MINIMUM_SHOOTING_DISTANCE = 2.59; //m
    public static double GRAVITY = 9.8; //m/s^2
    public static double HUB_HEIGHT = 2.641; //m
  }

  public static final class DriveConstants{
    public static final double OUTPUT_MIN = 0.0;
    public static final double OUTPUT_MAX = 1;
    public static final double kP_DRIVE = 0.008;
    public static final double ANGLE_THRESHOLD = 0.05;
    public static final double TURN_FACTOR = 0.75;
    public static final double LOW_DPI = 0.35;
  }

  public static final class AutoConstants {

    // Drive
    public static double kS = 0.1; // 0.390; //0.130 //0.402
    public static double kS_CONCRETE = 0.27;
    public static double kV = 3.59;    
    public static double kV_CONCRETE  = 3.;
    public static double kA = 0.458;
    public static double kRSQUARED = 0.945;
    public static double kTRACKWIDTH = 0.5842; //0.728585410;

    
    public static double kP_POSITION = 4.13;
    public static double kD_POSITION = 1850.0;
    public static double kP_VELOCITY = 1.8;
    public static double kD_VELOCITY = 0.0;

    public static double kP_POS_TEST1 = 5.63;
    public static double kV_POS_TEST1 = 2610.0;
    public static double kP_VEL_TEST1 = 2.53;
    public static double kV_VEL_TEST1 = 0;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Turn
    public static double TURN_KP = 0.12;
    public static double TURN_KI = 0.0;
    public static double TURN_KD = 0.0;
    public static double TURN_KF = 0.5;
    public static final double MIN_INPUT = -180.0f;
    public static final double MAX_INPUT = 180.0f;
    public static final double MIN_INGL = -1.0;
    public static final double MAX_INGL = 1.0;
    public static final double TOLERANCE = 0.3;
    public static final double COUNTS_PER_REV_SPARK = 42;
    public static final double COUNTS_PER_REV_775 = 1024;

    // Control Panel

    //Shooter
    public static double kVtoRPM = 0;
  }

  public static final class AutoConstantsFinal {

    // Drive
    public static double kS = 0;
    public static double kS_CONCRETE = 0.27;
    public static double kV = 3.59;
    public static double kA = 0.458;
    public static double kRSQUARED = 0.945;
    public static double kTRACKWIDTH = 0.5842; //0.728585410;

    public static double kP_POSITION = 4.13;
    public static double kD_POSITION = 1850.0;
    public static double kP_VELOCITY = 1.8;
    public static double kD_VELOCITY = 0.0;

    public static double kP_POS_TEST1 = 5.63;
    public static double kV_POS_TEST1 = 2610.0;
    public static double kP_VEL_TEST1 = 2.53;
    public static double kV_VEL_TEST1 = 0;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Turn
    public static double TURN_KP = 0.05;
    public static double TURN_KI = 0.0;
    public static double TURN_KD = 0.0;
    public static final double MIN_INPUT = -180.0f;
    public static final double MAX_INPUT = 180.0f;
    public static final double MIN_INGL = -1.0;
    public static final double MAX_INGL = 1.0;
    public static final double TOLERANCE = 0.3;
    public static final double COUNTS_PER_REV_SPARK = 42;
    public static final double COUNTS_PER_REV_775 = 1024;

    // Control Panel

    //Shooter
    public static double kVtoRPM = 0;
  }

  public static final class OIConstants {
    public static final int mainStickPort = 0;
    public static final int firstStickPort = 1;
    public static final int secondStickPort = 2;
    public static final int thirdStickPort = 3;

  }  

  public static final class IntakeConstants{
    public static double DEGREES_PER_TICK = 14.567;
  }
}