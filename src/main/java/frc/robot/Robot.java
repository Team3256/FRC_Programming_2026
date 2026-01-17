// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.NT4PublisherNoFMS;
import frc.robot.utils.PhoenixUtil;
import java.io.File;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends LoggedRobot {

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    super();
    SignalLogger.enableAutoLogging(false);
    RobotController.setBrownoutVoltage(4.75);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    if (Constants.FeatureFlags.kAdvKitEnabled) {
      configureAdvantageKit();
    }

    Runtime.getRuntime()
            .gc(); // gc is a blocking call; robot constructor will not initialize until this is
    // finished. this will cause "No Robot Code" until gc is finished.

  }

  private void configureAdvantageKit() {
    if (isReal()) {
      if (Constants.Logging.kLogToUSB) {
        // Note: By default, the WPILOGWriter class writes to a USB stick (at the path
        // of /U/logs) when running on the roboRIO. A FAT32 (sadly not exFAT, which is
        // the generally better format) formatted USB stick must be connected to one of
        // the roboRIO USB ports.
        Logger.addDataReceiver(new WPILOGWriter("/U/wpilogs"));
      } else {
        File usbLoc = new File("/home/lvuser/wpilogs");
        if (!usbLoc.exists()) {
          usbLoc.mkdirs();
          System.out.println("USB directory created at " + usbLoc.getAbsolutePath());
        }
        Logger.addDataReceiver(
                new WPILOGWriter("/home/lvuser/wpilogs")); // ensure this directory exists
        // advantage kit should be created before driverStationConnected()
      }
      Logger.addDataReceiver(new NT4PublisherNoFMS()); // Publish data to NetworkTables
      // Enables power distribution logging
      new PowerDistribution(
              1, ModuleType.kRev); // Ignore this "resource leak"; it was the example code from docs
    } else {
      if (Constants.Logging.kAdvkitUseReplayLogs) {
        setUseTiming(false); // Run as fast as possible
        String logPath =
                LogFileUtil
                        .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        // Save outputs to a new log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      } else {
        Logger.addDataReceiver(new NT4Publisher());
      }
    }

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME); // Set a metadata value
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("SerialNumber", RobotController.getSerialNumber());

    Logger.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    LoggedTracer.reset();
    PhoenixUtil.refreshAll();
    LoggedTracer.record("Phoenix Refresh");
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void driverStationConnected() {
    if (DriverStation.isFMSAttached()) {
      SignalLogger.start();
      SignalLogger.writeString("MatchStatus", "MatchNotStarted");
      SignalLogger.writeString("MatchType", DriverStation.getMatchType().toString());
      SignalLogger.writeString("MatchNumber", String.valueOf(DriverStation.getMatchNumber()));
      SignalLogger.writeString("EventName", DriverStation.getEventName());
      SignalLogger.writeString("ReplayNum", ((Integer) DriverStation.getReplayNumber()).toString());
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // no auto command, uses trigger
    SignalLogger.writeString("MatchStatus", "Auto");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    super.autonomousExit();
    SignalLogger.writeString("MatchStatus", "AutoEnded");
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    SignalLogger.writeString("MatchStatus", "Teleop");
  }

  @Override
  public void teleopExit() {
    super.teleopExit();
    SignalLogger.writeString("MatchStatus", "TeleopEnded");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}