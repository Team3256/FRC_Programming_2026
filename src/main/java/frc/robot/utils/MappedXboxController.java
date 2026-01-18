// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

public class MappedXboxController extends CommandXboxController {

  protected Map<String, String> buttonMap = new HashMap<>();

  public final String name;

  public MappedXboxController(int port, String name) {
    super(port);
    this.name = name;
  }

  public void labelButton(String button, String label) {
    buttonMap.put(button, label);
  }

  public Trigger leftBumper(String commandDescription) {
    labelButton("leftBumper", commandDescription);
    return this.leftBumper();
  }

  public Trigger rightBumper(String commandDescription) {
    labelButton("rightBumper", commandDescription);
    return this.rightBumper();
  }

  public Trigger leftStick(String commandDescription) {
    labelButton("leftStick", commandDescription);
    return this.leftStick();
  }

  public Trigger rightStick(String commandDescription) {
    labelButton("rightStick", commandDescription);
    return this.rightStick();
  }

  public Trigger a(String commandDescription) {
    labelButton("a", commandDescription);
    return this.a();
  }

  public Trigger b(String commandDescription) {
    labelButton("b", commandDescription);
    return this.b();
  }

  public Trigger x(String commandDescription) {
    labelButton("x", commandDescription);
    return this.x();
  }

  public Trigger y(String commandDescription) {
    labelButton("y", commandDescription);
    return this.y();
  }

  public Trigger start(String commandDescription) {
    labelButton("start", commandDescription);
    return this.start();
  }

  public Trigger back(String commandDescription) {
    labelButton("back", commandDescription);
    return this.back();
  }

  public Trigger leftTrigger(double threshold, String commandDescription) {
    labelButton("leftTrigger", commandDescription);
    return this.leftTrigger(threshold);
  }

  public Trigger leftTrigger(String commandDescription) {
    return this.leftTrigger(0.5, commandDescription);
  }

  public Trigger rightTrigger(double threshold, String commandDescription) {
    return this.rightTrigger(threshold);
  }

  public Trigger rightTrigger(String commandDescription) {
    return this.rightTrigger(0.5, commandDescription);
  }

  public double getTriggerAxes() {
    if (this.getLeftTriggerAxis() != 0 && this.getRightTriggerAxis() == 0) {
      return -getLeftTriggerAxis();
    }
    if (this.getRightTriggerAxis() != 0 && this.getLeftTriggerAxis() == 0) {
      return getRightTriggerAxis();
    } else if (this.getLeftTriggerAxis() != 0 && this.getRightTriggerAxis() != 0) {
      return 0.0;
    } else {
      return 0.0;
    }
  }

  public Trigger povRight(String commandDescription) {
    labelButton("povRight", commandDescription);
    return this.povRight();
  }

  public Trigger povLeft(String commandDescription) {
    labelButton("povLeft", commandDescription);
    return this.povLeft();
  }

  public Trigger povUp(String commandDescription) {
    labelButton("povUp", commandDescription);
    return this.povUp();
  }

  public Trigger povDown(String commandDescription) {
    labelButton("povDown", commandDescription);
    return this.povDown();
  }

  public Trigger povUpRight(String commandDescription) {
    labelButton("povUpRight", commandDescription);
    return this.povUpRight();
  }

  public Trigger povUpLeft(String commandDescription) {
    labelButton("povUpLeft", commandDescription);
    return this.povUpLeft();
  }

  public Trigger povDownRight(String commandDescription) {
    labelButton("povDownRight", commandDescription);
    return this.povDownRight();
  }

  public Trigger povDownLeft(String commandDescription) {
    labelButton("povDownLeft", commandDescription);
    return this.povDownLeft();
  }

  public Trigger pov(int angle, String commandDescription) {
    labelButton("POV " + angle, commandDescription);
    return this.pov(angle);
  }

  public static void dumpControllerMap(
      MappedXboxController controller1, MappedXboxController controller2) {
    try {
      Files.write(
          Paths.get("./scripts/map.json"),
          getControllerMap(controller1, controller2).getBytes(StandardCharsets.UTF_8));
      System.out.println("File written successfully!");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public static String getControllerMap(
      MappedXboxController controller1, MappedXboxController controller2) {
    Gson gson = new Gson();
    Map<String, Map<String, String>> buttonMap = new HashMap<>();
    buttonMap.put(controller1.name, controller1.buttonMap);
    buttonMap.put(controller2.name, controller2.buttonMap);
    return gson.toJson(buttonMap);
  }
}