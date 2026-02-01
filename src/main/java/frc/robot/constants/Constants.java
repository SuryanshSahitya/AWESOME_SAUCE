package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.LoggedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;


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
    public static final double unjammingspeed = -0.3;
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
  



    /** Gravity feedforward gain */
  public static final double pkG = 0.0; // NEEDS TUNING

  /** Static friction feedforward gain */
  public static final double pkS = 0.0; // NEEDS TUNING

  /** Proportional gain */
  public static final double pkP = 0.0; // NEEDS TUNING

  /** Derivative gain */
  public static final double pkD = 0.0; // NEEDS TUNING

  public static final double pivot_MOTION_MAGIC_CRUISE_VELOCITY = 0.0; // NEEDS SETTING

  /** Motion Magic acceleration in rotations per second² */
  public static final double pivot_MOTION_MAGIC_ACCELERATION = 0.0; // NEEDS SETTING
  public static final double MOTION_MAGIC_JERK = 0.0;



 //NEEDS TO BE TUNED FOR INTAKE PIVOT POSITIONS
  public static final double pivotUp = 0.0;
  public static final double pivotDown = 0.0;

  private IntakeConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
    }
  public static final class FlywheelConstants {


      private static FlywheelStates currentFlywheelState = FlywheelStates.OFF;

  // ==================== Shooting Speeds ====================

  /** How fast to spin for shooting (rotations per second) */
  public static final double SHOOTING_SPEED_RPS = 25.0;

  /** Slow speed for amp scoring (rotations per second) */
  public static final double amp_speed = 5.0;

  /** Fast speed for far field shooting (rotations per second) */
  public static final double far_speed = 35.0;

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
  public static final double MOTION_MAGIC_ACCELERATION = 2000.0;



  
  // ==================== Hood PID Control Values ====================

  /** Static friction compensation */
  public static final double hoodkS = 0.0;

  /** Velocity feedforward (predicts voltage needed for a speed) */
  public static final double hoodkV = 0.125;

  /** Proportional gain (corrects speed errors) */
  public static final double hoodkP = 0.0;

  // ==================== Motion Magic (Speed Limits) ====================

  /** Maximum speed the flywheel can reach (rotations per second) */
  public static final double hood_MOTION_MAGIC_CRUISE_VELOCITY = 50.0;

  /** How fast the flywheel can speed up (rotations per second²) */
  public static final double hood_MOTION_MAGIC_ACCELERATION = 30.0;


      public static enum FlywheelStates {
        OFF,
        CHARGING,
        READY,
        REVERSE
      }

  public static void setFlywheelState(FlywheelStates newState) {
    currentFlywheelState = newState;
    LoggedCommands.log("Robot state updated to: " + newState);
  }

  public static FlywheelStates getFlywheelState() {
    return currentFlywheelState;
  }


  private FlywheelConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
public static final class SpindexerConstants {

  // ==================== Shooting Speeds ====================

  /** How fast to spin for shooting (rotations per second) */
  public static final double SHOOTING_SPEED_RPS = 25.0;

  /** Slow speed for amp scoring (rotations per second) */
  public static final double amp_speed = 5.0;

  /** Fast speed for far field shooting (rotations per second) */
  public static final double far_speed = 35.0;

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
  public static final double MOTION_MAGIC_ACCELERATION = 2000.0;



  
  // ==================== Hood PID Control Values ====================

  /** Static friction compensation */
  public static final double spindexerkS = 0.0;

  /** Velocity feedforward (predicts voltage needed for a speed) */
  public static final double spindexerkV = 0.125;

  /** Proportional gain (corrects speed errors) */
  public static final double spindexerkP = 0.0;

  // ==================== Motion Magic (Speed Limits) ====================

  /** Maximum speed the flywheel can reach (rotations per second) */
  public static final double spindexer_MOTION_MAGIC_CRUISE_VELOCITY = 50.0;

  /** How fast the flywheel can speed up (rotations per second²) */
  public static final double spindexer_MOTION_MAGIC_ACCELERATION = 30.0;

  private static SpindexerStates currentSpindexerState = SpindexerStates.OFF;

      public static enum SpindexerStates {
        OFF,
        ROTATING
      }

  public static void setSpindexerState(SpindexerStates SnewState) {
    currentSpindexerState = SnewState;
    LoggedCommands.log("Robot state updated to: " + SnewState);
  }

  public static SpindexerStates getSpindexerState() {
    return currentSpindexerState;
  }


  private SpindexerConstants() {
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

public static final class SwerveConstants {
  // Physical properties
        public static final double kTrackWidth = Units.inchesToMeters(18.5);
        public static final double kTrackLength = Units.inchesToMeters(18.5);
        public static final double kRobotWidth = Units.inchesToMeters(25 + 3.25 * 2);
        public static final double kRobotLength = Units.inchesToMeters(25 + 3.25 * 2);
        public static final double kMaxLinearSpeed = Units.feetToMeters(15.5);
        public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        public static final double kDriveGearRatio = 6.75; // 6.75:1 SDS MK4 L2 ratio
        public static final double kSteerGearRatio = 12.8; // 12.8:1

        public static final double kDriveDistPerPulse = kWheelCircumference / 1024 / kDriveGearRatio;
        public static final double kSteerRadPerPulse = 2 * Math.PI / 1024;

        public enum ModuleConstants {
            FL( // Front left
                    1, 0, 0, 1, 1, 2, 3, 0, kTrackLength / 2, kTrackWidth / 2),
            FR( // Front Right
                    2, 2, 4, 5, 3, 6, 7, 0, kTrackLength / 2, -kTrackWidth / 2),
            BL( // Back Left
                    3, 4, 8, 9, 5, 10, 11, 0, -kTrackLength / 2, kTrackWidth / 2),
            BR( // Back Right
                    4, 6, 12, 13, 7, 14, 15, 0, -kTrackLength / 2, -kTrackWidth / 2);

            public final int moduleNum;
            public final int driveMotorID;
            public final int driveEncoderA;
            public final int driveEncoderB;
            public final int steerMotorID;
            public final int steerEncoderA;
            public final int steerEncoderB;
            public final double angleOffset;
            public final Translation2d centerOffset;

            private ModuleConstants(
                    int moduleNum,
                    int driveMotorID,
                    int driveEncoderA,
                    int driveEncoderB,
                    int steerMotorID,
                    int steerEncoderA,
                    int steerEncoderB,
                    double angleOffset,
                    double xOffset,
                    double yOffset) {
                this.moduleNum = moduleNum;
                this.driveMotorID = driveMotorID;
                this.driveEncoderA = driveEncoderA;
                this.driveEncoderB = driveEncoderB;
                this.steerMotorID = steerMotorID;
                this.steerEncoderA = steerEncoderA;
                this.steerEncoderB = steerEncoderB;
                this.angleOffset = angleOffset;
                centerOffset = new Translation2d(xOffset, yOffset);
            }
        }

        // Feedforward
        // Linear drive feed forward
        public static final SimpleMotorFeedforward kDriveFF =
                new SimpleMotorFeedforward( // real
                        0.25, // Voltage to break static friction
                        2.5, // Volts per meter per second
                        0.3 // Volts per meter per second squared
                        );
        // Steer feed forward
        public static final SimpleMotorFeedforward kSteerFF =
                new SimpleMotorFeedforward( // real
                        0.5, // Voltage to break static friction
                        0.25, // Volts per radian per second
                        0.01 // Volts per radian per second squared
                        );

        // PID
        public static final double kDriveKP = 1;
        public static final double kDriveKI = 0;
        public static final double kDriveKD = 0;

        public static final double kSteerKP = 20;
        public static final double kSteerKI = 0;
        public static final double kSteerKD = 0.25;
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

        public static final String kCameraName = "YOUR CAMERA NAME";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  private VisionConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

}
