// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.utils.DisableSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends DisableSubsystem {
  private static final double minAngle = Units.degreesToRadians(0.0); // 270
  private static final double maxAngle = Units.degreesToRadians(0.0);
  private static final double trackOverlapMargin = Units.degreesToRadians(10);
  private static final double trackCenterRads = (minAngle + maxAngle) / 2;
  private static final double trackMinAngle = trackCenterRads - Math.PI - trackOverlapMargin;
  private static final double trackMaxAngle = trackCenterRads + Math.PI + trackOverlapMargin;

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIO.TurretIOOutputs outputs = new TurretIO.TurretIOOutputs();

  @AutoLogOutput private ShootState shootState = ShootState.ACTIVE_SHOOTING;
  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocity = 0.0;
  private double lastGoalAngle = 0.0;

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert disconnected =
      new Alert("Turret motor disconnected!", Alert.AlertType.kWarning);
  private BooleanSupplier coastOverride = () -> false;

  SlewRateLimiter profile = new SlewRateLimiter(SwerveConstants.MaxSpeed); //TODO: verify
  private double turretOffset;

  public Turret(TurretIO turretIO) {
    this.turretIO = turretIO;
  }

  public void periodic() {
    turretIO.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // TODO: motor disconnected check

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      outputs.mode = TurretIO.TurretIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = TurretIO.TurretIOOutputMode.COAST;
      }
    }

    // Update profile constraints
    //
    //    if (maxVelocity.hasChanged(hashCode())) { // TODO: get robot max speed
    //      profile = new SlewRateLimiter(maxVelocity.get());
    //    }

    // Reset profile when disabled
    if (DriverStation.isDisabled()) {
      profile.reset(getPosition());
      lastGoalAngle = getPosition();
    }

    // Publish position
    RobotState.getInstance()
        .addTurretObservation(
            new RobotState.TurretObservation(Timer.getTimestamp(), new Rotation2d(getPosition())));
  }

  //TODO: add odom checker
  public void periodicAfterScheduler() {
    // Delay tracking math until after the RobotState has been updated
    if (DriverStation.isEnabled()) {
      Rotation2d robotAngle = RobotState.getInstance().getRotation();
      double robotAngularVelocity =
          RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;

      Rotation2d robotRelativeGoalAngle = goalAngle.minus(robotAngle);
      double robotRelativeGoalVelocity = goalVelocity - robotAngularVelocity;

      boolean hasBestAngle = false;
      double bestAngle = 0;
      double minLegalAngle =
          switch (shootState) {
            case ACTIVE_SHOOTING -> minAngle;
            case TRACKING -> trackMinAngle;
          };
      double maxLegalAngle =
          switch (shootState) {
            case ACTIVE_SHOOTING -> maxAngle;
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

      double setpoint = profile.calculate(MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle));
      Logger.recordOutput("Turret/GoalPositionRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", robotRelativeGoalVelocity);
      Logger.recordOutput("Turret/SetpointPositionRad", setpoint);
      Logger.recordOutput("Turret/SetpointVelocityRadPerSec", robotRelativeGoalVelocity);

      outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
      outputs.position = setpoint - turretOffset;
      outputs.velocity = robotRelativeGoalVelocity;
      outputs.kP = kP.get();
      outputs.kD = kD.get();
    }

    // Apply outputs
    turretIO.applyOutputs(outputs);
  }

  private void setFieldRelativeTarget(Rotation2d angle, double velocity) {
    this.goalAngle = angle;
    this.goalVelocity = velocity;
  }

  private void zero() {
    turretOffset = -inputs.positionRads;
  }

  @AutoLogOutput(key = "Turret/MeasuredPositionRad")
  public double getPosition() {
    return inputs.positionRads + turretOffset;
  }

  @AutoLogOutput(key = "Turret/MeasuredVelocityRadPerSec")
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public Command runTrackTargetCommand() { // TODO: need shot calculator
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
          setShootState(ShootState.TRACKING);
        });
  }

  public Command runTrackTargetActiveShootingCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters(); // TODO: merge shot calc
          setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
          setShootState(ShootState.ACTIVE_SHOOTING);
        });
  }

  public Command runFixedCommand(Supplier<Rotation2d> angle, DoubleSupplier velocity) {
    return run(
        () -> {
          setFieldRelativeTarget(angle.get(), velocity.getAsDouble());
          setShootState(ShootState.TRACKING);
        });
  }

  public Command zeroCommand() {
    return runOnce(this::zero).ignoringDisable(true);
  }

  public enum ShootState {
    ACTIVE_SHOOTING,
    TRACKING
  }
}
