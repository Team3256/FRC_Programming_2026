// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooterpivot;

import static frc.robot.subsystems.shooterpivot.ShooterPivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterPivot extends DisableSubsystem {

  private final ShooterPivotIO shooterPivotIO;
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  public final Trigger reachedPosition = new Trigger(this::reachedPosition);

  private double reqPosition = 0.0;

  private boolean atGoalPosition = false;

  // Turret specific states
  public enum ShootState {
    ACTIVE_SHOOTING,
    TRACKING;
  }

  // @Getter @Setter
  @AutoLogOutput private ShootState shootState = ShootState.ACTIVE_SHOOTING;
  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocity = 0.0;
  private double lastGoalAngle = 0.0;

  TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              shooterPivotMaxRotationalVelocity, shooterPivotMaxRotationalAcceleration));
  private State setpoint = new State();
  private double shooterPivotOffset;
  private boolean shooterPivotZeroed = false;

  private State getSetpoint() {
    return setpoint;
  }

  // TODO: Check this
  private void setShootState(ShootState state) {
    this.shootState = state;
  }

  public ShooterPivot(boolean enabled, ShooterPivotIO shooterPivotIO) {
    super(enabled);

    this.shooterPivotIO = shooterPivotIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    shooterPivotIO.updateInputs(inputs);
    Logger.processInputs("ShooterPivot", inputs);

    Logger.recordOutput(this.getClass().getSimpleName() + "/reqPosition", reqPosition);

    // Update lastGoalAngle & reset setpoint
    if (DriverStation.isDisabled()) {
      setpoint = new State(inputs.positionRads, 0.0);
      lastGoalAngle = getPosition();
    }

    // TODO: add turret position publication

    LoggedTracer.record("Shooter Pivot");
  }

  @Override
  public void periodicAfterScheduler() {
    // Delay tracking math until after the RobotState has been updated & turret zeroed
    if (DriverStation.isEnabled() && shooterPivotZeroed) {
      Rotation2d robotAngle = RobotState.getInstance().getRotation();
      double robotAngularVelocity =
          RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;

      Rotation2d robotRelativeGoalAngle = goalAngle.minus(robotAngle);
      double robotRelativeGoalVelocity = goalVelocity - robotAngularVelocity;

      boolean hasBestAngle = false;
      double bestAngle = 0;
      double minLegalAngle =
          switch (shootState) {
            case ACTIVE_SHOOTING -> minAngle.getRadians();
            case TRACKING -> trackMinAngle;
          };
      double maxLegalAngle =
          switch (shootState) {
            case ACTIVE_SHOOTING -> maxAngle.getRadians();
            case TRACKING -> trackMaxAngle;
          };
      for (int i = -2; i < 3; i++) {
        double potentialSetpoint = robotRelativeGoalAngle.getRadians() + Math.PI * 2.0 * i;
        if (potentialSetpoint < minLegalAngle || potentialSetpoint > maxLegalAngle) {
          continue;
        } else {
          if (!hasBestAngle) {
            bestAngle = potentialSetpoint;
            hasBestAngle = true;
          }
          if (Math.abs(lastGoalAngle - potentialSetpoint) < Math.abs(lastGoalAngle - bestAngle)) {
            bestAngle = potentialSetpoint;
          }
        }
      }
      lastGoalAngle = bestAngle;

      State goalState =
          new State(
              MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle), robotRelativeGoalVelocity);

      setpoint = profile.calculate(0.02, setpoint, goalState);
      atGoalPosition =
          EqualsUtil.epsilonEquals(bestAngle, setpoint.position)
              && EqualsUtil.epsilonEquals(robotRelativeGoalVelocity, setpoint.velocity);
      Logger.recordOutput("Turret/GoalPositionRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", robotRelativeGoalVelocity);
      Logger.recordOutput("Turret/SetpointPositionRad", setpoint.position);
      Logger.recordOutput("Turret/SetpointVelocityRadPerSec", setpoint.velocity);

      outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
      outputs.position = setpoint.position - turretOffset;
      outputs.velocity = setpoint.velocity;
      outputs.kP = kP.get();
      outputs.kD = kD.get();
    }
  }

  // TODO: SETTERS
  public Command setAbsPosition(double position) {
    return setPosition(() -> position);
  }

  public Command setPosition(DoubleSupplier position) {
    return this.run(
        () -> {
          reqPosition = position.getAsDouble();
          shooterPivotIO.setPosition(reqPosition);
        });
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> shooterPivotIO.setVoltage(voltage));
  }

  private void setFieldRelTarget(Rotation2d angle, double vel) {
    this.goalAngle = angle;
    this.goalVelocity = vel;
  }

  // TODO: GETTERS
  public double getPosition() {
    return inputs.shooterPivotMotorPosition + shooterPivotOffset;
  }

  public double getVelocity() {
    return inputs.shooterPivotMotorVelocity;
  }

  // TODO: ZEROES
  private void zero() {
    shooterPivotOffset = -inputs.shooterPivotMotorPosition; // TODO: convert this value to heading
    shooterPivotZeroed = true;
  }

  public Command zeroCommand() {
    return runOnce(this::zero).ignoringDisable(true);
  }

  public Command off() {
    return this.runOnce(shooterPivotIO::off).withName("off");
  }

  // TODO: TRACKING
  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setFieldRelTarget(params.turretAngle(), params.turretVelocity());
          setShootState(ShootState.TRACKING);
        });
  }

  public Command runTrackTargetActiveShootingCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setFieldRelTarget(params.turretAngle(), params.turretVelocity());
          setShootState(ShootState.ACTIVE_SHOOTING);
        });
  }

  public Command runFixedCommand(Supplier<Rotation2d> angle, DoubleSupplier velocity) {
    return run(
        () -> {
          setFieldRelTarget(angle.get(), velocity.getAsDouble());
          setShootState(ShootState.TRACKING);
        });
  }

  public boolean reachedPosition() {
    atGoalPosition = true;
    return Util.epsilonEquals(inputs.shooterPivotMotorPosition, reqPosition, 0.01);
  }
}
