// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants.FlywheelConstants;
import frc.robot.constants.Constants.SpindexerConstants;
import frc.robot.constants.Constants.SpindexerConstants.SpindexerStates;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.LoggedCommands;
import frc.robot.utils.TalonFXUtil;

@Logged
public class Spindexer extends SubsystemBase {

  // Main motor that spins the flywheel (device ID 21)
  protected final TalonFX spinner = new TalonFX(21);
  private final TalonFX passer = new TalonFX(23);

  // Controller for spinning the flywheel at a target speed
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  // How close the speed needs to be to count as "at target"
  private final AngularVelocity tolerance = RotationsPerSecond.of(SpindexerConstants.VELOCITY_TOLERANCE_RPS);

  // Configuration settings for the flywheel motor
  protected TalonFXConfiguration config = new TalonFXConfiguration();
  private TalonFXConfiguration hood_config = new TalonFXConfiguration();

  public Spindexer() {
    // Coast mode: Flywheel can spin freely by hand when disabled
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set motor direction: positive power = counterclockwise spin
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Control values from FlywheelConstants
    config.Slot0.kS = SpindexerConstants.kS; // Static friction
    config.Slot0.kV = SpindexerConstants.kV; // Velocity feedforward
    config.Slot0.kP = SpindexerConstants.kP; // Proportional gain

    // Speed limits from FlywheelConstants
    config.MotionMagic.MotionMagicCruiseVelocity = SpindexerConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = SpindexerConstants.MOTION_MAGIC_ACCELERATION;

    // Apply configuration with retries
    if (TalonFXUtil.applyConfigWithRetries(spinner, config, 2)) {
      Robot.telemetry().log("Flywheel/Config", true);
    } else {
      Robot.telemetry().log("Flywheel/Config", false);
    }

    hood_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Set motor direction: positive power = counterclockwise spin
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Control values from FlywheelConstants
    hood_config.Slot0.kS = SpindexerConstants.spindexerkS; // Static friction
    hood_config.Slot0.kV = SpindexerConstants.spindexerkV; // Velocity feedforward
    hood_config.Slot0.kP = SpindexerConstants.spindexerkP; // Proportional gain

    // Speed limits from FlywheelConstants
    hood_config.MotionMagic.MotionMagicCruiseVelocity = SpindexerConstants.spindexer_MOTION_MAGIC_CRUISE_VELOCITY;
    hood_config.MotionMagic.MotionMagicAcceleration = SpindexerConstants.spindexer_MOTION_MAGIC_ACCELERATION;



    if (TalonFXUtil.applyConfigWithRetries(passer, hood_config, 2)) {
      Robot.telemetry().log("Flywheel Hood/Config", true);
    } else {
      Robot.telemetry().log("Flywheel Hood/Config", false);
    }



  }

  @Override
  public void periodic() {
    // No periodic updates needed - control is entirely feedforward/feedback

    if(getspinnerVelocity() <= 5) {
      SpindexerConstants.setSpindexerState(SpindexerStates.OFF);
    } else if (getspinnerVelocity() > 5) {
      SpindexerConstants.setSpindexerState(SpindexerStates.ROTATING);
    }

    SmartDashboard.putString("Spindexer State",SpindexerConstants.getSpindexerState().name());
    SmartDashboard.putNumber("Spinner Speed/Velocity", getspinnerVelocity());
    SmartDashboard.putNumber("Passer Velocity", passer.getVelocity().getValueAsDouble());

  }

  public Command runSpinnerCommand(double speed) {
    return LoggedCommands.run("Running Spinner",() -> runSpinner(speed));
  }
  public Command runPasserCommand(double speed) {
    return LoggedCommands.run("Running Passer",() -> runPasser(speed));
  }


  public void runSpinner(double speed) {
    spinner.set(speed);
  }

  public void runPasser(double speed) {
    passer.set(speed);
  }

  public double getspinnerVelocity(){
    return spinner.getVelocity().getValueAsDouble();
  }

  /**
   * Command to stop the flywheel.
   *
   * @return Command that stops the flywheel
   */
  public Command stopCommand() {
    return LoggedCommands.run("Stopping All Spindexer Movement",() -> stop());
  }
  // Stop the flywheel motors (private to enforce Command-based control flow)
  private void stop() {
    spinner.stopMotor();
    passer.stopMotor();
  }
}