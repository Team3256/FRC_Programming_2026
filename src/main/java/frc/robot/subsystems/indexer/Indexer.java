// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

// TODO: FIll

public class Indexer extends DisableSubsystem {

  private final IndexerIO indexerIO;
  private final IndexerIOInputsAutoLogged indexerIOAutoLogged = new IndexerIOInputsAutoLogged();

  private final Timer highCurrentTimer = new Timer();
  private boolean isAutoUnjamming = false;
  private final Timer unjamTimer = new Timer();

  public Indexer(boolean enabled, IndexerIO indexerIO) {
    super(enabled);
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    indexerIO.updateInputs(indexerIOAutoLogged);
    Logger.processInputs("indexer", indexerIOAutoLogged);

    double statorCurrent = indexerIOAutoLogged.indexerMotor1StatorCurrent;

    if (isAutoUnjamming) {
      if (unjamTimer.hasElapsed(IndexerConstants.autoUnjamDuration)) {
        isAutoUnjamming = false;
        indexerIO.off();
        highCurrentTimer.stop();
        highCurrentTimer.reset();
      }
    } else {
      if (statorCurrent >= IndexerConstants.autoUnjamCurrentThreshold) {
        if (!highCurrentTimer.isRunning()) {
          highCurrentTimer.start();
        }
        if (highCurrentTimer.hasElapsed(IndexerConstants.autoUnjamCurrentDuration)) {
          isAutoUnjamming = true;
          unjamTimer.restart();
          indexerIO.setVoltage(-IndexerConstants.autoUnjamVoltage);
        }
      } else {
        highCurrentTimer.stop();
        highCurrentTimer.reset();
      }
    }

    LoggedTracer.record("Indexer");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> indexerIO.setVoltage(voltage)).finallyDo(indexerIO::off);
  }

  public Command setVelocity(double velocity) {
    return this.run(() -> indexerIO.setVelocity(velocity)).finallyDo(indexerIO::off);
  }

  public Command startShooting() {
    return setVoltage(IndexerConstants.indexVolt);
  }

  public Command off() {
    return this.runOnce(indexerIO::off);
  }
}
