package frc.robot.subsystems.superstructure
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.ghrobotics.lib.wrappers.hid.*
import frc.robot.subsystems.superstructure.Elevator

object Superstructure : FalconSubsystem() {

    fun goToPreset(elevatorHeightPreset : SIUnit <Meter> , proximalAnglePreset : SIUnit <Radian> , wristAnglePreset : SIUnit <Radian>) {

        var elevater = Elevator.master.encoder.position
        var proximal = Proximal.ProximalMaster.encoder.position
        var wrist = Wrist.wristMotor.encoder.position

        if(elevater > 40.inches && elevatorHeightPreset > 40.inches){
            //if high and going higher
            //dose not matter no risk of hitting things
            Proximal.ProximalPreset(proximalAnglePreset)
            ElevatorPresets.elevatorGoToPreset(elevatorHeightPreset)
            Wrist.wristPreset(wristAnglePreset)
        }
        // if high and going low
        else if(elevater > 40.inches){
            //proximal then wrist then elevator
            SequentialCommandGroup(Proximal.ProximalPreset(proximalAnglePreset), Wrist.wristPreset(wristAnglePreset), ElevatorPresets.elevatorGoToPreset(elevatorHeightPreset)).schedule()
        }
        //low and going lower/higher
        else {
            //elevator then proximal then wrist
            SequentialCommandGroup(ElevatorPresets.elevatorGoToPreset(elevatorHeightPreset), Proximal.ProximalPreset(proximalAnglePreset), Wrist.wristPreset(wristAnglePreset)).schedule()
        }


    }





    class cargoPresets() {

        //    SequentialCommandGroup(Proximal.ProximalPreset(ProximalAngle = 90.degrees),
//    Wrist.wristPreset(WristAngle = 10.degrees),
//    ElevatorPresets.elevatorGoToPreset(height = 5.inches)).schedule()
//
        fun low() {

        }

        fun mid() {

        }

        fun high() {

        }
    }

    class hatchPresets() {

        fun low() {

        }

        fun mid() {

        }

        fun high() {

        }

    }
}