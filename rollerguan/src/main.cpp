#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-3, 2, -1}, pros::MotorGearset::blue);    	
pros::MotorGroup right_mg({10, -9, 8}, pros::MotorGearset::blue);  
pros::Motor top(-11);
pros::Motor bottom(-12);
pros::adi::Pneumatics tongue('E', false);
pros::adi::Pneumatics centerupper('G', false);
pros::adi::Pneumatics wing('H', true);
pros::adi::Pneumatics pushdown('D', false);
pros::adi::Pneumatics clamp('F', false);
pros::Rotation rotation_horizontal(16);
pros::Rotation rotation_vertical(4);	
lemlib::Drivetrain drivetrain(left_mg, right_mg, 11.5, lemlib::Omniwheel::NEW_4, 343, 2);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		              	(pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
		int dir = master.get_analog(ANALOG_LEFT_Y);    
		int turn = master.get_analog(ANALOG_RIGHT_X); 
		left_mg.move(dir + turn);                   
		right_mg.move(dir - turn);      
		
		//intake
		if(master.get_digital(DIGITAL_R1))
		{
			bottom.move(127);
		}
		else if(master.get_digital(DIGITAL_R2))
		{
			bottom.move(-127);
		}
		else
		{
			bottom.move(0);

		//outtake
		}
		if(master.get_digital(DIGITAL_L1))
		{
			top.move(127);
			bottom.move(127);
		}
		else if(master.get_digital(DIGITAL_L2)){
			top.move(70);
			bottom.move(127);
			centerupper.extend();
		}
		else{
			top.move(0);
			centerupper.retract();
		}
		//pneumatics
		if(master.get_digital_new_press(DIGITAL_B))
		{
			tongue.toggle();
		}
		if(master.get_digital(DIGITAL_Y)){
			wing.retract();
		}
		else{
			wing.extend();
		}
		if(master.get_digital_new_press(DIGITAL_RIGHT)){
			clamp.toggle();
		}
		if (master.get_digital_new_press(DIGITAL_DOWN)){
			pushdown.toggle();
		}
			
		pros::delay(20);                              
	}
}