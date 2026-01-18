// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.sotm;

public class BallConstants {
  public final double mass;
  public final double radius;
  public final double area;

  public final double rho;
  public final double cd;
  public final double clGain;
  public final double clMax;

  public final double gravity;
  public final double spinDecayTau;

  public BallConstants(
      double mass,
      double radius,
      double rho,
      double cd,
      double clGain,
      double clMax,
      double gravity,
      double spinDecayTau) {

    this.mass = mass;
    this.radius = radius;
    this.area = Math.PI * radius * radius;

    this.rho = rho;
    this.cd = cd;
    this.clGain = clGain;
    this.clMax = clMax;

    this.gravity = gravity;
    this.spinDecayTau = spinDecayTau;
  }
}
