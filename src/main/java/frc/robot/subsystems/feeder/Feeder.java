// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

// TODO: FIll

public class Feeder extends DisableSubsystem {

  private final FeederIO feederIO;
  private final FeederIOInputsAutoLogged feederIOAutoLogged = new FeederIOInputsAutoLogged();

  public Feeder(boolean enabled, FeederIO indexerIO) {
    super(enabled);
    this.feederIO = indexerIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    feederIO.updateInputs(feederIOAutoLogged);
    Logger.processInputs("indexer", feederIOAutoLogged);

    LoggedTracer.record("Indexer");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> feederIO.setVoltage(voltage)).finallyDo(feederIO::off);
  }

  public Command setVelocity(double velocity) {
    return this.run(() -> feederIO.setVelocity(velocity)).finallyDo(feederIO::off);
  }

  public Command off() {
    return this.runOnce(feederIO::off);
  }
}
