// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.LoggedRobot;

public class TurretIOSim extends TurretIOTalonFX {

  private final SingleJointedArmSim turretSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          TurretConstants.TurretSim.turretSimGearing,
          TurretConstants.TurretSim.jkGMetersSquared,
          TurretConstants.TurretSim.turretLength.in(Meters),
          TurretConstants.TurretSim.minAngle.getRadians(),
          TurretConstants.TurretSim.maxAngle.getRadians(),
          true,
          TurretConstants.TurretSim.startingAngle.getRadians());

  private TalonFXSimState turretSimState;

  public TurretIOSim() {
    super();
    turretSimState = super.getMotor().getSimState();
    turretSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(TurretIO.TurretIOInputs inputs) {

    turretSimState = super.getMotor().getSimState();
    turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    turretSimModel.setInputVoltage(turretSimState.getMotorVoltage());
    turretSimModel.update(LoggedRobot.defaultPeriodSecs);
    turretSimState.setRawRotorPosition(
        Units.radiansToRotations(turretSimModel.getAngleRads())
            * TurretConstants.TurretSim.turretSimGearing);
    turretSimState.setRotorVelocity(
        Units.radiansToRotations(turretSimModel.getVelocityRadPerSec())
            * TurretConstants.TurretSim.turretSimGearing);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(turretSimModel.getCurrentDrawAmps()));

    super.updateInputs(inputs);
    // SimMechs.getInstance().updatePivot(Radians.of(turretSimModel.getAngleRads()));
  }
}
