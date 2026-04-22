// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class Indexer extends DisableSubsystem {

  private final IndexerIO indexerIO;
  private final IndexerIOInputsAutoLogged indexerIOAutoLogged = new IndexerIOInputsAutoLogged();

  private final Trigger jammed =
      new Trigger(
          () ->
              indexerIOAutoLogged.indexerMotor1StatorCurrent
                  >= IndexerConstants.autoUnjamCurrentThreshold);

  public Indexer(boolean enabled, IndexerIO indexerIO) {
    super(enabled);
    this.indexerIO = indexerIO;

    jammed.debounce(IndexerConstants.autoUnjamCurrentDuration).onTrue(this.unjam());
  }

  @Override
  public void periodic() {
    super.periodic();
    indexerIO.updateInputs(indexerIOAutoLogged);
    Logger.processInputs("indexer", indexerIOAutoLogged);

    LoggedTracer.record("Indexer");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> indexerIO.setVoltage(voltage)).finallyDo(indexerIO::off);
  }

  public Command setVelocity(double velocity) {
    return this.run(() -> indexerIO.setVelocity(velocity)).finallyDo(indexerIO::off);
  }

  public Command startShooting() {
    return setVoltage(IndexerConstants.indexVolt).finallyDo(indexerIO::off);
  }

  public Command unjam() {
    return setVoltage(IndexerConstants.autoUnjamVoltage)
        .withTimeout(IndexerConstants.autoUnjamDuration)
        .andThen(startShooting());
  }

  public Command off() {
    return this.runOnce(indexerIO::off);
  }
}
