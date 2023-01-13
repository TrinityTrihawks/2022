// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpikeLights extends SubsystemBase {
  /** Creates a new SpikeLights. */
  private static SpikeLights subsystemInst = null;
  private final Relay blueSpikeRelay = new Relay(0);
  private final Relay whiteSpikeRelay = new Relay(1);
  private boolean blueStatus = false;
  private boolean whiteStatus = false;
  // private static double dotMs = 500;
  // private static double dashMs = 1000;

  public static SpikeLights getInstance() {
    if (subsystemInst == null) {
      subsystemInst = new SpikeLights();
    }
    return subsystemInst;
  }

  public SpikeLights() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleBlueSpike() {
    if (blueStatus) {
      blueSpikeOff();
    } else {
      blueSpikeOn();
    }
  }

  public void toggleWhiteSpike() {
    if (whiteStatus) {
      whiteSpikeOff();
    } else {
      whiteSpikeOn();
    }
  }

  public void blueSpikeOn() {
    blueSpikeRelay.setDirection(Direction.kForward);
    blueSpikeRelay.set(Value.kOn);
    blueStatus = true;
  }

  public void blueSpikeOff() {
    blueSpikeRelay.set(Value.kOff);
    blueStatus = false;
  }

  public void whiteSpikeOn() {
    whiteSpikeRelay.setDirection(Direction.kForward);
    whiteSpikeRelay.set(Value.kOn);
    whiteStatus = true;
  }

  public void whiteSpikeOff() {
    whiteSpikeRelay.set(Value.kOff);
    whiteStatus = false;
  }
}
