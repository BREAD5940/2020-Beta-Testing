package frc.robot.subsystems.superstructure
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
//yeet delete ^^^^^
import frc.robot.subsystems.climb.ClimbSubsystem
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.inInches
import org.ghrobotics.lib.mathematics.units.inches

object Superstructure : FalconSubsystem() {

    fun goToPreset(elevatorHeightPreset : SIUnit <Meter> , proximalAnglePreset : SIUnit <Radian> , wristAnglePreset : SIUnit <Radian>, habYeetPreset : SIUnit <Meter>)  {

        var elevater = Elevator.master.encoder.position
        var proximal = Proximal.ProximalMaster.encoder.position
        var wrist = Wrist.wristMotor.encoder.position
        var habyeet = ClimbSubsystem.habClimberStilt.encoder.position


        if(elevater > 40.inches && elevatorHeightPreset > 40.inches){
            //if high and going higher
            //dose not matter no risk of hitting things
            Sequential(Proximal.ProximalPreset(proximalAnglePreset),
            ElevatorPresets.elevatorGoToPreset(elevatorHeightPreset),
            Wrist.WristPreset(wristAnglePreset), ClimbSubsystem.StiltSend(habYeetPreset)).schedule()
        }
        // if high and going low
        else if(elevater > 40.inches){
            //proximal then wrist then elevator
            SequentialCommandGroup(Proximal.ProximalPreset(proximalAnglePreset), Wrist.WristPreset(wristAnglePreset), ElevatorPresets.elevatorGoToPreset(elevatorHeightPreset)).schedule()
        }
        //low and going lower/higher heheh
        else {
            //elevator then proximal then wrist
            SequentialCommandGroup(ElevatorPresets.elevatorGoToPreset(elevatorHeightPreset), Proximal.ProximalPreset(proximalAnglePreset), Wrist.WristPreset(wristAnglePreset)).schedule()
        }


    }

    //private fun SequentialCommandGroup(proximalPreset: Proximal.ProximalPreset, elevatorGoToPreset: ElevatorPresets.elevatorGoToPreset, wristPreset: Wrist.WristPreset, habClimberStilt: Unit): SequentialCommandGroup {




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