package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

val hatchMotor = FalconSRX(51, DefaultNativeUnitModel)
val cargoMotor = FalconSRX(50, DefaultNativeUnitModel)
var cargo = false
object Intake : FalconSubsystem() {

    val hatchMotor = FalconSRX(51, DefaultNativeUnitModel)
    val cargoMotor = FalconSRX(50, DefaultNativeUnitModel)
    val solenoid = FalconDoubleSolenoid(0,1, 8)




    fun cargoIntake() {
        cargoMotor.talonSRX.configPeakOutputForward(0.8)
        hatchMotor.talonSRX.configPeakOutputForward(0.8)
        solenoid.state = FalconSolenoid.State.Forward
    }
    fun hatchIntake() {
        hatchMotor.talonSRX.configPeakOutputForward(0.8)
        solenoid.state = FalconSolenoid.State.Reverse

    }
    fun cargoOutake() {
        cargoMotor.talonSRX.configPeakOutputForward(-0.8)
        hatchMotor.talonSRX.configPeakOutputForward(-0.8)
        solenoid.state = FalconSolenoid.State.Reverse

    }
    fun hatchOutake() {
        hatchMotor.talonSRX.configPeakOutputForward(-0.8)
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

//TODO my code yetus deletused and now there is no code, i re write