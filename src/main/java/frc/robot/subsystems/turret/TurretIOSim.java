// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class TurretIOSim extends TurretIOTalonFX {

  private final LinearSystem<N1, N1, N1> flywheelSystem =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1),
          TurretConstants.SimulationConstants.kMomentOfInertia,
          TurretConstants.SimulationConstants.turretSimGearing);

  private final FlywheelSim turretSimModel =
      new FlywheelSim(flywheelSystem, DCMotor.getKrakenX60(1));

  private final TalonFXSimState turretSimState;

  public TurretIOSim() {
    super();
    turretSimState = super.getMotor().getSimState();
    turretSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    // Update battery voltage
    turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    // Update physics models
    turretSimModel.setInput(turretSimState.getMotorVoltage());

    turretSimModel.update(LoggedRobot.defaultPeriodSecs);

    double motor1Rps = turretSimModel.getAngularVelocityRPM() / 60 * TurretConstants.SimulationConstants.turretSimGearing;
    turretSimState.setRotorVelocity(motor1Rps);
    turretSimState.addRotorPosition(motor1Rps * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(turretSimModel.getCurrentDrawAmps()));

    super.updateInputs(inputs);

    SimMechs.getInstance()
        .updateTurret(
            Rotations.of(
                inputs.turretMotorPosition));
  }
}
