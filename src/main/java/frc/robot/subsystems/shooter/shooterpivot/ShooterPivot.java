// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.shooterpivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterPivot extends DisableSubsystem {

  private final ShooterPivotIO shooterPivotIO;
  private final ShooterPivotIOInputsAutoLogged shooterPivotIOInputsAutoLogged =
      new ShooterPivotIOInputsAutoLogged();

  public final Trigger reachedPosition = new Trigger(this::reachedPosition);

  //    private final SimpleMotorFeedforward m_shooterFeedforward =
  //            new SimpleMotorFeedforward(
  //                    ShooterPivotConstants.kSVolts,
  // ShooterPivotConstants.kVVoltSecondsPerRotation);

  private final PIDController pivotRotationController =
      new PIDController(
          ShooterPivotConstants.trackingP,
          ShooterPivotConstants.trackingI,
          ShooterPivotConstants.trackingD);

  private double reqPosition = 0.0;

  public ShooterPivot(boolean enabled, ShooterPivotIO shooterPivotIO) {
    super(enabled);

    this.shooterPivotIO = shooterPivotIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    shooterPivotIO.updateInputs(shooterPivotIOInputsAutoLogged);
    Logger.processInputs("ShooterPivot", shooterPivotIOInputsAutoLogged);

    Logger.recordOutput(this.getClass().getSimpleName() + "/reqPosition", reqPosition);

    LoggedTracer.record("ShooterPivot");
  }

  public Command setPosition(Rotation2d position) {
    return setPosition(() -> position);
  }

  public Command setPosition(Supplier<Rotation2d> position) {
    return this.run(
        () -> {
          reqPosition = position.get().getRotations();
          shooterPivotIO.setPosition(reqPosition);
        });
  }

  public Command setPositionFieldRelative(Rotation2d position, CommandSwerveDrivetrain swerve) {
    return this.run(
        () ->
            shooterPivotIO.setPosition(
                position.minus(swerve.getState().Pose.getRotation()).getRotations()));
  }

  public double getPosition() {
    return shooterPivotIOInputsAutoLogged.shooterPivotMotorPosition;
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> shooterPivotIO.setVoltage(voltage));
  }

  public double getVoltage() {
    return shooterPivotIOInputsAutoLogged.shooterPivotMotorVoltage;
  }

  public Command trackTarget(
      ShotCalculator calc, CommandSwerveDrivetrain drivetrain, Translation3d target) {

    // return ParallelCommandGroup FEED FORWARD

    calc.setTarget(new Pose3d(target, Rotation3d.kZero));
    return run(
        () -> {
          pivotRotationController.calculate(
              shooterPivotIOInputsAutoLogged.shooterPivotMotorPosition,
              calc.getCurrentPivotAngle().getRotations());
        });
  }

  public Command zero() {
    return this.runOnce(shooterPivotIO::zero);
  }

  public Command off() {
    return this.runOnce(shooterPivotIO::off).withName("off");
  }

  public boolean reachedPosition() {
    return Util.epsilonEquals(
        shooterPivotIOInputsAutoLogged.shooterPivotMotorPosition, reqPosition, 0.01);
  }
}
