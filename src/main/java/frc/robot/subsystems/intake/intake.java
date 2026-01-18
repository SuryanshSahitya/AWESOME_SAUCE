// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants.*;
import frc.robot.utils.LoggedCommands;
import frc.robot.utils.TalonFXUtil;

@Logged
public class intake extends SubsystemBase {
  // Connect to the "canivore" CAN bus (communication network for motors)
  private final CANBus canivore = new CANBus("canivore");

  // Main motor that moves the arm (device ID 31)
  public final TalonFX intakeL = new TalonFX(31);
  public final TalonFX intakeR = new TalonFX(4);
  public final TalonFX intakePivot = new TalonFX(6);

  public static CANcoder encoder = new CANcoder(32);
  
  public static String up_or_down;
  public static double pivotPosition = 0;

  // Configuration settings for the arm motor
  public TalonFXConfiguration config = new TalonFXConfiguration();
  private TalonFXConfiguration pivotconfig = new TalonFXConfiguration();

  final MotionMagicVoltage pivotIntakeVoltage = new MotionMagicVoltage(0);


  // Controller for moving the arm to specific positions


  public intake() {
    // Coast mode: Motor can be moved by hand when disabled (easier for testing)
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set motor direction: positive power = counterclockwise rotation
    //config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Automatically fights gravity using math

    // Control values from ArmConstants (TODO: CRITICAL - Tune these on the real robot!)
    config.Slot0.kG = IntakeConstants.kG; // Gravity compensation
    config.Slot0.kS = IntakeConstants.kS; // Static friction
    config.Slot0.kP = IntakeConstants.kP; // Proportional gain (speed of correction)
    config.Slot0.kD = IntakeConstants.kD; // Derivative gain (smoothness)

    // Motion limits from ArmConstants (TODO: CRITICAL - Set non-zero values!)
    config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY; // Max speed
    config.MotionMagic.MotionMagicAcceleration = IntakeConstants.MOTION_MAGIC_ACCELERATION; // How fast to speed up
    // intakeL.getConfigurator().apply(config);
    // intakeR.getConfigurator().apply(config);
    TalonFXUtil.applyConfigWithRetries(intakeR, config);
    TalonFXUtil.applyConfigWithRetries(intakeL, config);



    pivotconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotconfig.Slot0.kG = IntakeConstants.pkG; // Gravity compensation
    pivotconfig.Slot0.kS = IntakeConstants.pkS; // Static friction
    pivotconfig.Slot0.kP = IntakeConstants.pkP; // Proportional gain (speed of correction)
    pivotconfig.Slot0.kD = IntakeConstants.pkD;
    pivotconfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.pivot_MOTION_MAGIC_CRUISE_VELOCITY; // Max speed
    pivotconfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.pivot_MOTION_MAGIC_ACCELERATION; // How fast to speed up
    pivotconfig.MotionMagic.MotionMagicJerk = IntakeConstants.MOTION_MAGIC_JERK; // How fast to speed up
    //intakePivot.getConfigurator().apply(pivotconfig);
    TalonFXUtil.applyConfigWithRetries(intakePivot, pivotconfig);

  }

  @Override
  public void periodic() {
    // No periodic updates needed - control is entirely feedforward/feedback
    DogLog.log("IntakePivot/Position", getIntakePivotPosition());
    DogLog.log("IntakePivot/TorqueCurrent", getIntakePivotTorqueCurrent());
    DogLog.log("IntakePivot/Velocity",  getIntakePivotVelocity());
  }


  /**
   * Command to stop the arm motor.
   *
   * @return Command that stops the arm
   */
  public Command stopIntakeCommand() {
    return LoggedCommands.runOnce("Stopping Intake motors", () -> stopIntake());
  }

  // Stop the arm motor (private to enforce Command-based control flow)
  private void stopIntake() {
    intakeL.stopMotor();
    intakeR.stopMotor();
  }

 private void runIntake(double speed) {
    intakeL.set(speed);
    intakeR.set(-speed);
 }

 public Command runIntakeCommand(double speed) {
  return LoggedCommands.run("Running Intake",() -> runIntake(speed));
 }
 public Command unjammingIntakecCommand() {
  return LoggedCommands.run("Unjamming Intake", () -> runIntake(IntakeConstants.unjammingspeed));
 }


 public double getIntakePivotPosition() {
  return intakePivot.getPosition().getValueAsDouble();
 }

  public double getIntakePivotTorqueCurrent() {
  return intakePivot.getTorqueCurrent().getValueAsDouble();
 } 
 public double getIntakePivotVelocity() {
  return intakePivot.getVelocity().getValueAsDouble();
 }

 public void intakepiv(double position) {
  intakePivot.setControl(pivotIntakeVoltage.withPosition(position).withEnableFOC(true).withFeedForward(0.6));
 }

 public Command intakepivCommand(String position) {

  if (position.equalsIgnoreCase("up")) {
    up_or_down = "up";
    pivotPosition = IntakeConstants.pivotUp;
  } else if (position.equalsIgnoreCase("down")) {
    up_or_down = "down";
    pivotPosition = IntakeConstants.pivotDown;
  } else {
    up_or_down = "ERROR";
  }

  return LoggedCommands.run("Pivoting Intake to " + position + " position", () -> intakepiv(pivotPosition));
 }


//  public Command intakepivoitcmd(double postion){
//     return new Command(){
//         @Override
//         public void initialize(){
            
//         }
//         @Override
//         public void execute(){
//             intakePivot.setControl(pivotIntakeVoltage.withPosition(postion).withEnableFOC(true).withFeedForward(0.6));
//         }
//         @Override
//         public boolean isFinished(){
//             return false;
//         }
//         @Override
//         public void end(boolean interrupted){
     
//         }
//     };
//    }


}