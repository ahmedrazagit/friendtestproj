from vehicle_mechanics.control import run_with_controller
from parking_routines import ParallelParkController, PerpendicularParkController, LeaveParkController

def sequential_parking_routine():
    """
    Allow the user to choose which parking controller to run:
    1. Parallel Parking
    2. Leaving Parking
    3. Perpendicular Parking
    """
    print("Select a parking routine to execute:")
    print("1. Parallel Parking and Leave Parking")
    print("2. Perpendicular Parking")
    
    try:
        choice = int(input("Enter the number of your choice: "))
        
        if choice == 1:
            print("Executing Parallel Parking routine...")
            run_with_controller(ParallelParkController)
            run_with_controller(LeaveParkController)
        elif choice == 2:
            print("Executing Perpendicular Parking routine...")
            run_with_controller(PerpendicularParkController)
        else:
            print("Invalid choice. Please select a number between 1 and 3.")
    except ValueError:
        print("Invalid input. Please enter a number.")

if __name__ == '__main__':
    sequential_parking_routine()
