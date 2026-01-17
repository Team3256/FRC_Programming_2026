// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

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

public class ShooterIOSim extends ShooterIOTalonFX {

  private final DCMotor motor =
      ShooterConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1);

  private final LinearSystem<N1, N1, N1> leftFlywheelPlant =
      LinearSystemId.createFlywheelSystem(
          motor,
          ShooterConstants.SimulationConstants.kLeftGearingRatio,
          ShooterConstants.SimulationConstants.kLeftMomentOfInertia);

  private final LinearSystem<N1, N1, N1> rightFlywheelPlant =
      LinearSystemId.createFlywheelSystem(
          motor,
          ShooterConstants.SimulationConstants.kRightGearingRatio,
          ShooterConstants.SimulationConstants.kRightMomentOfInertia);

  private final FlywheelSim leftFlywheelSimModel = new FlywheelSim(leftFlywheelPlant, motor);

  private final FlywheelSim rightFlywheelSimModel = new FlywheelSim(rightFlywheelPlant, motor);

  private final TalonFXSimState shooterMotorSim;
  private final TalonFXSimState shooterMotorFollowerSim;

  public ShooterIOSim() {
    super();
    shooterMotorSim = super.getMotor().getSimState();
    shooterMotorFollowerSim = super.getFollowerMotor().getSimState();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Update battery voltage
    shooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    shooterMotorFollowerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // Update physics models
    leftFlywheelSimModel.setInput(shooterMotorSim.getMotorVoltage());
    leftFlywheelSimModel.update(LoggedRobot.defaultPeriodSecs);
    rightFlywheelSimModel.setInput(shooterMotorFollowerSim.getMotorVoltage());
    rightFlywheelSimModel.update(LoggedRobot.defaultPeriodSecs);

    double leftRps = leftFlywheelSimModel.getAngularVelocityRPM() / 60;
    shooterMotorSim.setRotorVelocity(leftRps);
    shooterMotorSim.addRotorPosition(leftRps * LoggedRobot.defaultPeriodSecs);
    double rightRps = rightFlywheelSimModel.getAngularVelocityRPM() / 60;
    shooterMotorFollowerSim.setRotorVelocity(rightRps);
    shooterMotorFollowerSim.addRotorPosition(rightRps * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            leftFlywheelSimModel.getCurrentDrawAmps(), rightFlywheelSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);
    
  }
}
