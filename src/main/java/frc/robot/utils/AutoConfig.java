package frc.robot.utils;

import choreo.auto.AutoRoutine;

import java.util.List;
import java.util.function.Supplier;

public class AutoConfig {
    public final String name;
    public final Supplier<AutoRoutine> routine;
    public final List<String> trajectories;

    public AutoConfig(String name, Supplier<AutoRoutine> routine, List<String> trajectories) {
        this.name = name;
        this.routine = routine;
        this.trajectories = trajectories;
    }
}