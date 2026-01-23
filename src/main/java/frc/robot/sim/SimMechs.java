// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.units.measure.Angle;

public final class SimMechs {

  public final LoggedMechanism2d mech =
      new LoggedMechanism2d(Constants.SimulationConstants.kDrivebaseWidth.in(Meters), 1);

  private static SimMechs instance = null;

  private SimMechs() {}

  public static SimMechs getInstance() {
    if (instance == null) {
      instance = new SimMechs();
    }
    return instance;
  }

  public void publishToNT() {
    Logger.recordOutput("RobotSim", mech);
  }

public void updateClimb(Angle of) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateClimb'");
}
}
