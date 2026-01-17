
package frc.robot.subsystems.intakerollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends DisableSubsystem {
  private final IntakeRollersIO intakeRollersIO;
  private final IntakeRollersIOInputsAutoLogged intakeIOAutoLogged =
      new IntakeRollersIOInputsAutoLogged();


  public IntakeRollers(boolean enabled, IntakeRollersIO intakeRollersIO) {
    super(enabled);
    this.intakeRollersIO = intakeRollersIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    intakeRollersIO.updateInputs(intakeIOAutoLogged);
    Logger.processInputs("IntakeRollers", intakeIOAutoLogged);

    LoggedTracer.record("IntakeRollers");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> intakeRollersIO.setVoltage(voltage))
        .finallyDo(intakeRollersIO::off);
  }

  public Command setVelocity(double velocity) {
    return this.run(() -> intakeRollersIO.setVelocity(velocity))
        .finallyDo(intakeRollersIO::off);
  }

  public Command off() {
    return this.runOnce(intakeRollersIO::off);
  }


}