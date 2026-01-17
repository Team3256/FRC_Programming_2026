// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends DisableSubsystem {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterIOAutoLogged = new ShooterIOInputsAutoLogged();
  private final LoggedTunableNumber shooterMotorVelocityInput =
      new LoggedTunableNumber("Shooter/MotorVelocity");
  private final LoggedTunableNumber shooterFollowerVelocityInput =
      new LoggedTunableNumber("Shooter/FollowerVelocity");


  public Shooter(boolean disabled, ShooterIO shooterIO) {
    super(disabled);
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    shooterIO.updateInputs(shooterIOAutoLogged);
    if (Constants.FeatureFlags.kTuningModeEnabled) {
      double velocity = shooterMotorVelocityInput.getOrUse(0);
      shooterIO.setShooterVelocity(velocity);
      shooterIO.setShooterFollowerVelocity(shooterFollowerVelocityInput.getOrUse(velocity));
    }
    Logger.processInputs(this.getClass().getSimpleName(), shooterIOAutoLogged);
  }

  public Command setVoltage(double voltage, double followerVoltage) {
    return this.run(
            () -> {
              shooterIO.setShooterVoltage(voltage);
              shooterIO.setShooterFollowerVoltage(followerVoltage);
            })
        .finallyDo(shooterIO::off);
  }

  public Command setVelocity(double velocity, double followerVelocity) {
    return this.run(
            () -> {
              shooterIO.setShooterVelocity(velocity);
              shooterIO.setShooterFollowerVelocity(followerVelocity);
            })
        .finallyDo(shooterIO::off);
  }


  public Command off() {
    return this.runOnce(shooterIO::off);
  }
}
