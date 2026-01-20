// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.networktables.NT4Publisher;

// subclass of NT4Publisher that does not publish to NT if FMS is attached.
// may alleviate some command overrun issues
public class NT4PublisherNoFMS extends NT4Publisher {
  public NT4PublisherNoFMS() {
    super();
  }

  @Override
  public void putTable(LogTable table) {
    if (Robot.isReal() && DriverStation.isFMSAttached()) {
      return;
    }
    super.putTable(table);
  }
}
