// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Notifier;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.DoubleConsumer;

/**
 * Steps mechanism physics sims faster than the robot loop. A TalonFX runs its position loop at
 * about 1 kHz, so a sim stepped once per 20 ms loop hands it a position that is up to 20 ms stale,
 * and gains tuned on the real mechanism (the hood's kP is 600) ring between the end stops. Each IO
 * sim registers its physics step here; its updateInputs then just reads the motor like on the
 * robot.
 */
public final class MechanismSimLoop {
  private static final List<DoubleConsumer> steps = new CopyOnWriteArrayList<>();
  private static Notifier notifier;

  private MechanismSimLoop() {}

  /** Runs {@code step} every sim period with the period in seconds, starting the loop if needed. */
  public static synchronized void register(DoubleConsumer step) {
    steps.add(step);
    if (notifier == null) {
      double period = SimConstants.kMechanismPeriod.in(Seconds);
      notifier = new Notifier(() -> steps.forEach(s -> s.accept(period)));
      notifier.setName("MechanismSim");
      notifier.startPeriodic(period);
    }
  }
}
