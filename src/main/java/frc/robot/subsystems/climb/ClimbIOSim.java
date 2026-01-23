// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class ClimbIOSim extends ClimbIOTalonFX {
  private final SingleJointedArmSim climbSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.sim.simGearing,
          ClimbConstants.sim.jkGMetersSquared,
          ClimbConstants.sim.climbLength.in(Meters),
          ClimbConstants.sim.minAngle.getRadians(),
          ClimbConstants.sim.maxAngle.getRadians(),
          true,
          ClimbConstants.sim.startingAngle.getRadians());
  private TalonFXSimState climbSimState;

  public ClimbIOSim() {
    super();
    climbSimState = super.getMotor().getSimState();
    climbSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    climbSimState = super.getMotor().getSimState();
    climbSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    climbSimModel.setInputVoltage(climbSimState.getMotorVoltage());
    climbSimModel.update(LoggedRobot.defaultPeriodSecs);
    climbSimState.setRawRotorPosition(
        Units.radiansToRotations(climbSimModel.getAngleRads()) * ClimbConstants.sim.simGearing);
    climbSimState.setRotorVelocity(
        Units.radiansToRotations(climbSimModel.getVelocityRadPerSec())
            * ClimbConstants.sim.simGearing);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(climbSimModel.getCurrentDrawAmps()));
    climbSimState.setRotorVelocity(
        RadiansPerSecond.of(climbSimModel.getVelocityRadPerSec()).in(RotationsPerSecond));
    super.updateInputs(inputs);
    SimMechs.getInstance().updateClimb(Radians.of(climbSimModel.getAngleRads()));
  }
}
