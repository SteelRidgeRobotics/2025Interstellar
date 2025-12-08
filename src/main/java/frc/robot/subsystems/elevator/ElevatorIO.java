package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {

    public boolean elevatorConnected = false;
    public double positionRads = 0.0;
    public double velocityRads = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrent = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setPosition(double position) {}
}
