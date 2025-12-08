package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  class PivotIOInputs {
    public boolean pivotConnected = false;
    public Rotation2d pivotAbsolutePosition = Rotation2d.kZero;
    public double positionRads = 0.0;
    public double velocityRads = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double statorCurrent = 0.0;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setPosition(Rotation2d rotation) {}
}
