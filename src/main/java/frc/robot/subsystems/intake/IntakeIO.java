package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean coralIntakeConnected = false;
    public double coralIntakeVelocityRadsPerSec = 0.0;
    public double coralIntakeAppliedVoltage = 0.0;
    public double coralIntakeCurrentAmps = 0.0;

    public boolean coral2IntakeConnected = false;
    public double coral2IntakeVelocityRadsPerSec = 0.0;
    public double coral2IntakeAppliedVoltage = 0.0;
    public double coral2IntakeCurrentAmps = 0.0;

    public boolean algaeIntakeConnected = false;
    public double algaeIntakeVelocityRadsPerSec = 0.0;
    public double algaeIntakeAppliedVoltage = 0.0;
    public double algaeIntakeCurrentAmps = 0.0;

    public boolean sensorSensed = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setIntakeOpenLoop(double output, boolean ignoreLimits) {}
}
