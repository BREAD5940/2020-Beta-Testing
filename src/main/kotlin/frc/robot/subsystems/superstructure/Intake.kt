package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid



object Intake : FalconSubsystem() {
    val hatchMotor = FalconSRX(51, DefaultNativeUnitModel)
    val cargoMotor = FalconSRX(50, DefaultNativeUnitModel)
    var cargo = false
    val solenoid = FalconDoubleSolenoid(0,1, 8)

    init{
        hatchMotor.talonSRX.configFactoryDefault()
        cargoMotor.talonSRX.configFactoryDefault()
        }

    fun cargoIntake() {
        cargoMotor.setDutyCycle(1.0)
        hatchMotor.setDutyCycle(1.0)
        solenoid.state = FalconSolenoid.State.Forward
    }
    fun hatchIntake() {
        hatchMotor.setDutyCycle(1.0)
        solenoid.state = FalconSolenoid.State.Reverse
    }
    fun cargoOutake() {
        //I made the motors only yeet insted of adjust because we only really yeet them
        cargoMotor.setDutyCycle(-1.0)
        hatchMotor.setDutyCycle(-1.0)
        solenoid.state = FalconSolenoid.State.Reverse
    }
    fun hatchOutake() {
        hatchMotor.setDutyCycle(-1.0)
        solenoid.state = FalconSolenoid.State.Reverse
    }
    fun stop() {
        cargoMotor.setNeutral()
        hatchMotor.setNeutral()
        solenoid.state = FalconSolenoid.State.Reverse
    }
    fun intake() {
        if (cargo){
            cargoIntake()
        }
        else{
            hatchIntake()
        }
    }
    fun outake() {
        if(cargo){
            cargoOutake()
        }
        else{
            hatchOutake()
        }
    }
}

