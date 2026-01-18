// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.hal.HALUtil;

public class StackJumper {
  private static boolean traceTime = false;

  public static String getCallerMethodName() {
    if (traceTime) {
      double d = HALUtil.getFPGATime();
      String methodName =
          StackWalker.getInstance()
              .walk(stream -> stream.skip(2).findFirst().get())
              .getMethodName();
      System.out.println(
          "Time to get method name: " + (HALUtil.getFPGATime() - d) + " ns for " + methodName);
      return methodName;
    } else {
      return StackWalker.getInstance()
          .walk(stream -> stream.skip(2).findFirst().get())
          .getMethodName();
    }
  }
}
