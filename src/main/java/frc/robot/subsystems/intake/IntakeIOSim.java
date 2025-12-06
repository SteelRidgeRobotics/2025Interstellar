package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
    // make sim motors (coral intake bundles the 2 motors together because they work in tandem)
    private static final DCMotor coralIntakeSIM = DCMotor.getKrakenX60(2);
    private static final DCMotor algaeIntakeSIM = DCMotor.getKrakenX60(1);

    private final DCMotorSim intakeSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                coralIntakeSIM, 0.01, Constants.IntakeConstants.GEAR_RATIO),
            coralIntakeSIM);

    private final DCMotorSim algaeSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                algaeIntakeSIM, 0.01, Constants.IntakeConstants.GEAR_RATIO),
            algaeIntakeSIM);

    private double intakeAppliedVolts = 0.0;

@Override
public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts,-12,12));
    intakeSim.update(0.02);

    inputs.algaeIntakeConnected = true;
    inputs.algaeIntakeVelocityRadsPerSec = algaeSim.getAngularVelocityRadPerSec();
    inputs.algaeIntakeAppliedVoltage = intakeAppliedVolts;
    inputs.algaeIntakeCurrentAmps = Math.abs(algaeSim.getCurrentDrawAmps());

    inputs.coralIntakeConnected = true;
    inputs.coralIntakeVelocityRadsPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.coralIntakeAppliedVoltage = intakeAppliedVolts;
    inputs.coralIntakeCurrentAmps = Math.abs(intakeSim.getCurrentDrawAmps());

    inputs.coral2IntakeConnected = true;
    inputs.coral2IntakeVelocityRadsPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.coral2IntakeAppliedVoltage = intakeAppliedVolts;
    inputs.coral2IntakeCurrentAmps = Math.abs(intakeSim.getCurrentDrawAmps());
    
} 

public void setIntakeOpenLoop(double output) {
    intakeAppliedVolts = output;
    }
}
