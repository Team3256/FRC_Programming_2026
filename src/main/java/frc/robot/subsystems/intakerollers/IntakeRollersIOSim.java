package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.sim.CANrangeSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
//import frc.robot.sim.SimMechs;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.LoggedRobot;

public class IntakeRollersIOSim extends IntakeRollersIOTalonFX {
  private final FlywheelSim rollerSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              IntakeRollersConstants.kUseFOC
                  ? DCMotor.getKrakenX60Foc(1)
                  : DCMotor.getKrakenX60(1),
              IntakeRollersConstants.SimulationConstants.rollerGearingRatio,
              IntakeRollersConstants.SimulationConstants.rollerMomentOfInertia),
              IntakeRollersConstants.kUseFOC
              ? DCMotor.getKrakenX60Foc(1)
              : DCMotor.getKrakenX60(1));

  private final TalonFXSimState motorSim;

  private final CANrangeSimState canRangeSimState;

  private final LoggedTunableNumber canRangeDistance =
      new LoggedTunableNumber("GIRCanRangeDistance", 10.0);

  public IntakeRollersIOSim() {
    super();
    motorSim = super.getIntakeRollerMotor().getSimState();
    canRangeSimState = super.getCanRange().getSimState();
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {

    // Update battery voltage
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    canRangeSimState.setSupplyVoltage(RobotController.getBatteryVoltage());


    // Update physics models
    rollerSimModel.setInput(motorSim.getMotorVoltage());
    rollerSimModel.update(LoggedRobot.defaultPeriodSecs);

    double motorRPS = rollerSimModel.getAngularVelocityRPM() / 60;
    motorSim.setRotorVelocity(motorRPS);
    motorSim.addRotorPosition(motorRPS * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);
  }
}