package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  // make sim motors (coral intake bundles the 2 motors together because they work in tandem)
  private static final DCMotor coralIntakeSim = DCMotor.getKrakenX60(2);
  private static final DCMotor algaeIntakeSim = DCMotor.getKrakenX60(1);

  private final DCMotorSim intakeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              coralIntakeSim, 0.01, Constants.IntakeConstants.GEAR_RATIO),
          coralIntakeSim);

  private final DCMotorSim algaeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              algaeIntakeSim, 0.01, Constants.IntakeConstants.GEAR_RATIO),
          algaeIntakeSim);

  private double coralIntakeAppliedVolts = 0.0;
  private double algaeIntakeAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.setInputVoltage(MathUtil.clamp(coralIntakeAppliedVolts, -12, 12));
    intakeSim.update(0.02);

    algaeSim.setInputVoltage(MathUtil.clamp(algaeIntakeAppliedVolts, -12, 12));
    algaeSim.update(0.02);

    inputs.algaeIntakeConnected = true;
    inputs.algaeIntakeVelocityRadsPerSec = algaeSim.getAngularVelocityRadPerSec();
    inputs.algaeIntakeAppliedVoltage = algaeIntakeAppliedVolts;
    inputs.algaeIntakeCurrentAmps = Math.abs(algaeSim.getCurrentDrawAmps());

    inputs.coralIntakeConnected = true;
    inputs.coralIntakeVelocityRadsPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.coralIntakeAppliedVoltage = coralIntakeAppliedVolts;
    inputs.coralIntakeCurrentAmps = Math.abs(intakeSim.getCurrentDrawAmps());

    inputs.coral2IntakeConnected = true;
    inputs.coral2IntakeVelocityRadsPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.coral2IntakeAppliedVoltage = coralIntakeAppliedVolts;
    inputs.coral2IntakeCurrentAmps = Math.abs(intakeSim.getCurrentDrawAmps());
  }

  @Override
  public void setCoralOpenLoop(double output, boolean ignoreLimits) {
    coralIntakeAppliedVolts = output;
  }

  @Override
  public void setAlgaeOpenLoop(double output, boolean ignoreLimits) {
    algaeIntakeAppliedVolts = output;
  }
}
