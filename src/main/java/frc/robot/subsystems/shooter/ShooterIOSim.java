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
import org.littletonrobotics.junction.LoggedRobot;

public class ShooterIOSim extends ShooterIOTalonFX {

  private final DCMotor motor =
      ShooterConstants.kUseFOC ? DCMotor.getKrakenX60Foc(2) : DCMotor.getKrakenX60(2);

  private final LinearSystem<N1, N1, N1> flywheelSystem =
      LinearSystemId.createFlywheelSystem(
          motor,
          ShooterConstants.SimulationConstants.kLeftGearingRatio,
          ShooterConstants.SimulationConstants.kLeftMomentOfInertia);

  private final FlywheelSim flywheelSim = new FlywheelSim(flywheelSystem, motor);

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
    flywheelSim.setInput(shooterMotorSim.getMotorVoltage());
    flywheelSim.update(LoggedRobot.defaultPeriodSecs);

    double motor1Rps = flywheelSim.getAngularVelocityRPM() / 60;
    shooterMotorSim.setRotorVelocity(motor1Rps);
    shooterMotorSim.addRotorPosition(motor1Rps * LoggedRobot.defaultPeriodSecs);
    double motor2Rps = flywheelSim.getAngularVelocityRPM() / 60;
    shooterMotorFollowerSim.setRotorVelocity(motor2Rps);
    shooterMotorFollowerSim.addRotorPosition(motor2Rps * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            flywheelSim.getCurrentDrawAmps()));
    super.updateInputs(inputs);

  }
}
