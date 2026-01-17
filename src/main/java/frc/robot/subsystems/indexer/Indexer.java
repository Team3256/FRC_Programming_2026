// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

// TODO: FIll

public class Indexer extends DisableSubsystem {

    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged indexerIOAutoLogged = new IndexerIOInputsAutoLogged();

    public Indexer(boolean enabled, IndexerIO IndexerIO) {
        super(enabled);
        this.indexerIO = IndexerIO;
    }

    @Override
    public void periodic() {
        super.periodic();
        IndexerIO.updateInputs(indexerIOAutoLogged);
        Logger.processInputs("indexer", indexerIOAutoLogged);

        LoggedTracer.record("Indexer");
    }

    public Command setVoltage(double voltage) {
        return this.run(() -> IndexerIO.setVoltage(voltage)).finallyDo(IndexerIO::off);
    }

    public Command setVelocity(double velocity) {
        return this.run(() -> IndexerIO.setVelocity(velocity)).finallyDo(IndexerIO::off);
    }

    public Command off() {
        return this.runOnce(IndexerIO::off);
    }
}