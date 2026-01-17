package frc.robot.constants;

public class Constants {
    public static final class PhysicalConstants {
        
  // ==================== Robot Physical Properties ====================

  /** Robot mass in pounds (used for wheel force calculations) */
  public static final double ROBOT_MASS_LBS = 125.0;

  /** Robot moment of inertia in kg·m² (used for rotational dynamics). If you don't know leave at 6*/
  public static final double MOMENT_OF_INERTIA_KG_M2 = 6.0;

  // ==================== Linear Motion Constraints ====================

  /** Maximum linear velocity in meters per second */
  public static final double MAX_LINEAR_VELOCITY_MPS = 3.0;

  /** Maximum linear acceleration in meters per second² */
  public static final double MAX_LINEAR_ACCELERATION_MPS2 = 3.0;

  // ==================== Angular Motion Constraints ====================

  /** Maximum angular velocity in radians per second (π rad/s = 0.5 rotations/sec) */
  public static final double MAX_ANGULAR_VELOCITY_RAD_S = Math.PI;

  /** Maximum angular acceleration in radians per second² (2π rad/s² = 1 rotation/sec²) */
  public static final double MAX_ANGULAR_ACCELERATION_RAD_S2 = 2 * Math.PI;

  // ==================== Path Following PID Gains ====================
  // Used by LinearPathRequest for feedback corrections

  /** X position PID proportional gain */
  public static final double X_CONTROLLER_KP = 10.0;

  /** Y position PID proportional gain */
  public static final double Y_CONTROLLER_KP = 10.0;

  /** Theta (rotation) PID proportional gain */
  public static final double THETA_CONTROLLER_KP = 7.0;

  private PhysicalConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
    }
    public static final class IntakeConstants{

    public static final double speed = 0.7;
    public static final double unjammingspeed = 0.3;
          // ==================== PID Control Values ====================
  // TODO: CRITICAL - These control how the arm moves! Test on real robot first!
  // Start with these safe values: kG=0.2 (fights gravity), kS=0.2 (overcomes friction),
  //                               kP=160 (speed of correction), kD=30 (smoothness)
  // If the arm jerks or moves too fast, make these numbers smaller.

  /** Gravity feedforward gain */
  public static final double kG = 0.0; // NEEDS TUNING

  /** Static friction feedforward gain */
  public static final double kS = 0.0; // NEEDS TUNING

  /** Proportional gain */
  public static final double kP = 0.0; // NEEDS TUNING

  /** Derivative gain */
  public static final double kD = 0.0; // NEEDS TUNING

  // ==================== Motion Magic (Speed Limits) ====================
  // TODO: CRITICAL - Set how fast the arm can move!
  // Recommended starting values: Max Speed=2 rotations/second, Acceleration=4 rotations/second²

  /** Motion Magic cruise velocity in rotations per second */
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0; // NEEDS SETTING

  /** Motion Magic acceleration in rotations per second² */
  public static final double MOTION_MAGIC_ACCELERATION = 0.0; // NEEDS SETTING

  private IntakeConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
    }
    public static final class FlywheelConstants {

  // ==================== Shooting Speeds ====================

  /** How fast to spin for shooting (rotations per second) */
  public static final double SHOOTING_SPEED_RPS = 25.0;

  /** Slow speed for amp scoring (rotations per second) */
  public static final double AMP_SPEED_RPS = 5.0;

  /** Fast speed for far field shooting (rotations per second) */
  public static final double FAR_SHOOTING_SPEED_RPS = 35.0;

  // ==================== Tolerances ====================

  /** How close the speed needs to be to count as "at target" (RPS) */
  public static final double VELOCITY_TOLERANCE_RPS = 0.25;

  // ==================== PID Control Values ====================

  /** Static friction compensation */
  public static final double kS = 0.0;

  /** Velocity feedforward (predicts voltage needed for a speed) */
  public static final double kV = 0.125;

  /** Proportional gain (corrects speed errors) */
  public static final double kP = 0.0;

  // ==================== Motion Magic (Speed Limits) ====================

  /** Maximum speed the flywheel can reach (rotations per second) */
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 100.0;

  /** How fast the flywheel can speed up (rotations per second²) */
  public static final double MOTION_MAGIC_ACCELERATION = 1000.0;

  private FlywheelConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
public static final class LEDConstants {
        /* IDs */
        public static final int leftCandle = 0;
        public static final int rightCandle = 1;
        /* CANbus */
        public static final String canBus = "rio";
        /* LED arrangement */
        public static final int startIdx = 8;
        public static final int numLEDs = 86;
        public static final int totalLEDs = startIdx + numLEDs;
        public static final double brightness = 1.00;
        /* Animations */
        // public static final Animation readyAnimation = new FireAnimation(1.0, 0.38, numLEDs, 0.8, 0.2, false, startIdx);
        // public static final Animation climbedAnimation = new RainbowAnimation(1.0, 0.7, numLEDs, false, startIdx);
        // public static final Animation climbingAnimation = new LarsonAnimation(255, 64, 0, 0, 0.85, numLEDs, BounceMode.Front, 7, startIdx);
        // public static final Animation endGameAnimation = new ColorFlowAnimation(255, 64, 0, 0, 0.85, numLEDs, ColorFlowAnimation.Direction.Forward, startIdx);
        // public static final Animation serviceModeAnimation = new ColorFlowAnimation(0, 25, 25, 0, 0.3, numLEDs, ColorFlowAnimation.Direction.Backward, startIdx);
        
        /* Misc */
        public static final double blinkRate = 0.2; // Regular blink rate
        public static final double errorBlinkRate = 0.1; // Blink rate for errors and warnings
        public static final double tempStateTime = 0.70; // How long for warnings and errors
        public static final double endGameNotifyStart = 20.0;
        public static final double endGameNotifyDuration = 4.0;   
    private LEDConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

/**
 * Settings for the Limelight vision camera.
 *
 * <p>Controls how much we trust the camera's position measurements:
 * <ul>
 *   <li>How accurate we expect measurements to be
 *   <li>How much to trust measurements (based on distance and number of tags)
 *   <li>Maximum and minimum trust levels
 * </ul>
 */
public static final class VisionConstants {

  // ==================== Base Trust Levels ====================

  /** Starting trust level for X/Y position per AprilTag seen (meters) */
  public static final double BASE_XY_STD_DEV = 0.5;

  /** Starting trust level for rotation per AprilTag seen (radians) */
  public static final double BASE_THETA_STD_DEV = 5;

  // ==================== Ambiguity Filtering ====================

  /** Maximum acceptable ambiguity for AprilTag detections (0.0 = perfect, 1.0 = completely ambiguous) */
  public static final double MAX_TAG_AMBIGUITY = 0.7;

  // ==================== Trust Level Limits ====================

  /** Most we'll ever trust X/Y measurements (meters) - very confident */
  public static final double MIN_XY_STD_DEV = 0.01;

  /** Least we'll ever trust X/Y measurements (meters) - not confident */
  public static final double MAX_XY_STD_DEV = 2.0;

  /** Most we'll ever trust rotation measurements (radians) - very confident */
  public static final double MIN_THETA_STD_DEV = 0.05;

  /** Least we'll ever trust rotation measurements (radians) - not confident */
  public static final double MAX_THETA_STD_DEV = 1.0;

  private VisionConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

}
