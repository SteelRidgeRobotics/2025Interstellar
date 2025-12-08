package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  private static final DCMotor ELEVATOR_GEARBOX = DCMotor.getKrakenX60Foc(2);
  private final DCMotorSim elevatorSim;

  private boolean closedLoop = true;

  private final ProfiledPIDController elevatorController =
      new ProfiledPIDController(
          ElevatorConstants.GAINS.kP / (2 * Math.PI),
          ElevatorConstants.GAINS.kI / (2 * Math.PI),
          ElevatorConstants.GAINS.kD / (2 * Math.PI),
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(ElevatorConstants.MM_CRUISE_VELOCITY),
              Units.rotationsToRadians(ElevatorConstants.MM_UPWARDS_ACCELERATION)));

  private double elevatorAppliedVolts;

  public ElevatorIOSim() {
    elevatorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ELEVATOR_GEARBOX, 0.01, ElevatorConstants.GEAR_RATIO),
            ELEVATOR_GEARBOX);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      elevatorAppliedVolts = elevatorController.calculate(elevatorSim.getAngularPositionRad());
    } else {
      elevatorController.reset(
          elevatorSim.getAngularPositionRad(), elevatorSim.getAngularVelocityRadPerSec());
    }

    elevatorSim.setInputVoltage(MathUtil.clamp(elevatorAppliedVolts, -12, 12));
    elevatorSim.update(0.02);

    inputs.elevatorConnected = true;
    inputs.positionRads = elevatorSim.getAngularPositionRad();
    inputs.velocityRads = elevatorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = elevatorAppliedVolts;
    inputs.statorCurrent = Math.abs(elevatorSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    elevatorAppliedVolts = output;
  }

  @Override
  public void setPosition(double position) {

    closedLoop = true;

    double pos = Units.rotationsToRadians(position);
    elevatorController.setGoal(pos);
  }
}
