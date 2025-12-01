package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs{
        public boolean topIntakeConnected = false;
        public double topIntakeVelocityRadsPerSec = 0.0;
        public double topIntakeAppliedVoltage = 0.0;
        public double topIntakeCurrentAmps = 0.0;

        public boolean bottomIntakeConnected = false;
        public double bottomIntakeVelocityRadsPerSec = 0.0;
        public double bottomIntakeAppliedVoltage = 0.0;
        public double bottomIntakeCurrentAmps = 0.0;

        public boolean sensorSensed = false;
    }
    
    default void updateInputs(IntakeIOInputs inputs) {}

    default void setIntakeOpenLoop(double output, boolean ignoreLimits) {}
}
