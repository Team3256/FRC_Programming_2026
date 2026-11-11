// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

// Adapted from Iron Maple 5516's CTRE-Swerve-MapleSim template (MIT license):
// https://github.com/Shenzhen-Robotics-Alliance/CTRE-Swerve-MapleSim
// Ported to the maple-sim 0.4 API.

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Drives a CTRE {@link com.ctre.phoenix6.swerve.SwerveDrivetrain} from a maple-sim physics
 * drivetrain, in place of CTRE's own {@code SimSwerveDrivetrain}. maple-sim works out where the
 * robot really goes (wheel slip, collisions with the field and other robots) and this class writes
 * the result back into the simulated TalonFXs, CANcoders and Pigeon, so CTRE's odometry sees it the
 * way it would on the robot. The robot's real position is {@link #mapleSimDrive}'s pose; the
 * drivetrain's own pose is the odometry estimate, which can drift from it like on a real field.
 */
public class MapleSimSwerveDrivetrain {
  private final Pigeon2SimState pigeonSim;
  public final SwerveDriveSimulation mapleSimDrive;

  /**
   * @param simPeriod how often {@link #update()} is called
   * @param robotMassWithBumpers mass of the robot, bumpers and battery included
   * @param bumperLengthX bumper size front to back (the robot's collision box)
   * @param bumperWidthY bumper size side to side
   * @param driveMotorModel drive motor, e.g. {@code DCMotor.getKrakenX60Foc(1)}
   * @param steerMotorModel steer motor
   * @param wheelCOF tire coefficient of friction
   * @param moduleLocations module locations in the order FL, FR, BL, BR
   * @param pigeon the drivetrain's Pigeon 2
   * @param modules the drivetrain's modules, from {@code SwerveDrivetrain#getModules()}
   * @param moduleConstants the module constants, in the same order
   */
  @SafeVarargs
  public MapleSimSwerveDrivetrain(
      Time simPeriod,
      Mass robotMassWithBumpers,
      Distance bumperLengthX,
      Distance bumperWidthY,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCOF,
      Translation2d[] moduleLocations,
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    this.pigeonSim = pigeon.getSimState();
    var moduleConfig =
        new SwerveModuleSimulationConfig(
            driveMotorModel,
            steerMotorModel,
            moduleConstants[0].DriveMotorGearRatio,
            moduleConstants[0].SteerMotorGearRatio,
            Volts.of(moduleConstants[0].DriveFrictionVoltage),
            Volts.of(moduleConstants[0].SteerFrictionVoltage),
            Meters.of(moduleConstants[0].WheelRadius),
            KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
            wheelCOF);
    DriveTrainSimulationConfig simulationConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMassWithBumpers)
            .withBumperSize(bumperLengthX, bumperWidthY)
            .withGyro(COTS.ofPigeon2())
            .withCustomModuleTranslations(moduleLocations)
            .withSwerveModule(() -> new SwerveModuleSimulation(moduleConfig));
    mapleSimDrive = new SwerveDriveSimulation(simulationConfig, new Pose2d());

    SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
    for (int i = 0; i < moduleSimulations.length; i++) {
      moduleSimulations[i].useDriveMotorController(
          new TalonFXMotorControllerSim(modules[i].getDriveMotor()));
      moduleSimulations[i].useSteerMotorController(
          new TalonFXMotorControllerWithRemoteCanCoderSim(
              modules[i].getSteerMotor(), modules[i].getEncoder()));
    }

    SimulatedArena.overrideSimulationTimings(simPeriod, 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
  }

  /**
   * Steps the whole arena (drivetrain, FUEL, hubs) and writes the result into the Pigeon.
   *
   * <p>The Pigeon reads maple-sim's gyro, which integrates the robot's rotation (with drift) like a
   * real one, rather than the robot's absolute heading. Teleporting the robot (resetPose) then
   * leaves the gyro alone, as on the real robot; feeding it the absolute heading made the yaw jump
   * right after CTRE had computed its offset, doubling the heading.
   */
  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    var gyro = mapleSimDrive.getGyroSimulation();
    pigeonSim.setRawYaw(gyro.getGyroReading().getMeasure());
    pigeonSim.setAngularVelocityZ(gyro.getMeasuredAngularVelocity());
  }

  /** Feeds a simulated TalonFX its rotor position and velocity and reads back its output. */
  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  /** Same as {@link TalonFXMotorControllerSim}, also feeding the module's CANcoder. */
  public static class TalonFXMotorControllerWithRemoteCanCoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);
      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  /**
   * Adjusts module constants so they work in simulation. Does nothing on the real robot, so the
   * real constants are never touched. Call it on the constants before the drivetrain is built.
   *
   * <ul>
   *   <li>Encoder offsets and motor/encoder inversions are zeroed: the simulated devices have no
   *       mounting offsets, and inverted drive motors break the drive PID in sim.
   *   <li>The couple ratio is zeroed: maple-sim doesn't model steer-to-drive coupling, so CTRE
   *       would otherwise "correct" odometry for coupling that never happened.
   *   <li>Friction voltages are clamped into the range maple-sim accepts.
   *   <li>Steer inertia is raised to 0.05 kg m^2. maple-sim steps the steer motor with plain Euler
   *       integration, which rings, or blows up, when the step is longer than the motor's back-EMF
   *       time constant (J R / (Kt Ke G^2): about 1.3 ms at the generated 0.01 with our 26:1 steer,
   *       6.5 ms at 0.05, against a 5 ms step). The module still swings just as fast, since the
   *       steer motor's free speed is what limits it.
   * </ul>
   */
  public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    if (RobotBase.isReal()) return moduleConstants;

    for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
      moduleConstant
          .withEncoderOffset(0)
          .withDriveMotorInverted(false)
          .withSteerMotorInverted(false)
          .withEncoderInverted(false)
          .withCouplingGearRatio(0)
          .withDriveFrictionVoltage(
              Volts.of(Math.min(Math.max(moduleConstant.DriveFrictionVoltage, 0.02), 0.3)))
          .withSteerFrictionVoltage(
              Volts.of(Math.min(Math.max(moduleConstant.SteerFrictionVoltage, 0.02), 0.5)))
          .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
    return moduleConstants;
  }
}
