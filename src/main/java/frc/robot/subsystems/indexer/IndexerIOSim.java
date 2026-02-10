// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

// TODO: FILL

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.intakerollers.IntakeRollersConstants;
import org.littletonrobotics.junction.LoggedRobot;

public class IndexerIOSim extends IndexerIOTalonFX {
  private FlywheelSim indexerSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              IntakeRollersConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
              IntakeRollersConstants.SimulationConstants.rollerGearingRatio,
              IntakeRollersConstants.SimulationConstants.rollerMomentOfInertia),
          IntakeRollersConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1));

  private final TalonFXSimState motorSim;

  public IndexerIOSim() {
    super();
    motorSim = super.getIndexerMotor().getSimState();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    // Update battery voltage
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update physics models
    indexerSimModel.setInput(motorSim.getMotorVoltage());
    indexerSimModel.update(LoggedRobot.defaultPeriodSecs);

    double motorRPS = indexerSimModel.getAngularVelocityRPM() / 60;
    motorSim.setRotorVelocity(motorRPS);
    motorSim.addRotorPosition(motorRPS * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(indexerSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);
  }
}
