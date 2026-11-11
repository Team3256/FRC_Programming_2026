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
import frc.robot.sim.MechanismSimLoop;
import frc.robot.sim.SimMechs;

public class TurretIOSim extends TurretIOTalonFX {

  private final LinearSystem<N1, N1, N1> turretSystem =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1),
          TurretConstants.SimulationConstants.kMomentOfInertia,
          TurretConstants.SimulationConstants.turretSimGearing);

  private final FlywheelSim turretSimModel = new FlywheelSim(turretSystem, DCMotor.getKrakenX60(1));

  private final TalonFXSimState turretSimState;

  public TurretIOSim() {
    super();
    turretSimState = super.getMotor().getSimState();
    turretSimState.Orientation = ChassisReference.Clockwise_Positive;
    MechanismSimLoop.register(this::step);
  }

  /** Steps the physics, faster than the robot loop so the TalonFX's position loop stays stable. */
  private void step(double dtSeconds) {
    turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    turretSimModel.setInput(turretSimState.getMotorVoltage());
    turretSimModel.update(dtSeconds);

    double rotorRps =
        turretSimModel.getAngularVelocityRPM()
            / 60
            * TurretConstants.SimulationConstants.turretSimGearing;
    turretSimState.setRotorVelocity(rotorRps);
    turretSimState.addRotorPosition(rotorRps * dtSeconds);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(turretSimModel.getCurrentDrawAmps()));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    super.updateInputs(inputs);
    SimMechs.getInstance().updateTurret(Rotations.of(inputs.turretMotorPosition));
  }
}
