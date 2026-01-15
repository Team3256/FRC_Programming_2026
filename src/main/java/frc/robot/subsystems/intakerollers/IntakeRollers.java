
package frc.robot.subsystems.intakerollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends DisableSubsystem {
  private final IntakeRollersIO groundIntakeRollersIO;
  private final IntakeRollersIOInputsAutoLogged intakeIOAutoLogged =
      new IntakeRollersIOInputsAutoLogged();

  public final Trigger coralIntakeIn =
      new Trigger(
          () ->
              (intakeIOAutoLogged.canRangeDistance
                  < GroundIntakeRollersConstants.canRangeInThreshold));

  public IntakeRollers(boolean enabled, IntakeRollersIO IntakeRollersIO) {
    super(enabled);
    this.groundIntakeRollersIO = IntakeRollersIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    groundIntakeRollersIO.updateInputs(intakeIOAutoLogged);
    Logger.processInputs("GroundIntake", intakeIOAutoLogged);

    Logger.recordOutput("GroundIntake/coral", coralIntakeIn.getAsBoolean());

    LoggedTracer.record("GroundIntake");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> groundIntakeRollersIO.setVoltage(voltage))
        .finallyDo(IntakeRollersIO::off);
  }

  public Command setVelocity(double velocity) {
    return this.run(() -> groundIntakeRollersIO.setVelocity(velocity))
        .finallyDo(IntakeRollersIO::off);
  }

  public Command off() {
    return this.runOnce(IntakeRollersIO::off);
  }

  public Command intakeCoral() {
    return setVoltage(IntakeRollersConstants.intakeVoltage)
        .until(coralIntakeIn)
        .finallyDo(IntakeRollersIO::off);
  }

  public double getDetectedCanRangeDistance() {
    return intakeIOAutoLogged.canRangeDistance;
  }

  public Command handoffCoral() {
    return setVoltage(IntakeRollersConstants.handoffVoltage)
        .finallyDo(IntakeRollersIO::off);
  }

  public Command outtakeL1() {
    return setVoltage(IntakeRollersConstants.l1OuttakeVoltage);
  }
}