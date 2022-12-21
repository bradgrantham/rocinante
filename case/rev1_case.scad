// 0, 0, 0 is left, front corner of *bottom* of board.
// therefore board is resting on YX plane

board_width = 150;
board_depth = 100;
board_thickness = 1.6;

shell_upper_half_space = 1;
shell_lower_half_space = 1;

max_component_height_above_board = 16.30 - board_thickness; // populated ESP-01.  CALIPERS INCLUDED BOARD
max_component_height_below_board = 10.74 - board_thickness; // Power barrel.  CALIPERS INCLUDED BOARD

shell_side_space = .5; // space between boards and box sides
shell_wall_thickness = 2;

// clockwise from left far corner

// micro_usb
microusb_left = 14.82;
microusb_right= 24.13;
microusb_top = 4.42; // from bottom of board

button_height = 8.72; // from bottom of board
button_diameter = 3.37;

// spring-button connectors for BOOT0, RESET
boot0_height = button_height;
boot0_center = [32.82, 78.39, 0] ; // from left, from front
boot0_diameter = button_diameter;

reset_height = button_height;
reset_center = [32.82, 67.51, 0] ; // from left, from front
reset_diameter = button_diameter;

// audio out connector
audio_out_center = [44.68, board_depth, 8.47]; // from left, from board top
audio_out_diameter = 8;

// audio in connector
audio_in_center = [66.59, board_depth, 8.47]; // from left, from board top
audio_in_diameter = 8;

// composite connector
comp_left = 80.94; // from left
comp_width = 11;
comp_bottom = 0 ; // from bottom of board
comp_top = 13.07; // from bottom of board

// VGA connector
vga_left = 110.5;
vga_right = 128.02;
vga_bottom = 3.81;
vga_top = 12.20;

// debug cable connector cutout

debug_front = 58.12; // from front of board
debug_back = 71.40; // from front of board
debug_thickness = .63; // need at least this much cut out for cable

// a little space for the PS/2 connector sticking out but no reason to cut out for the connector
ps2_is_proud = .87;

// controller 2
ctrl2_left = 106.05;
ctrl2_right = 137.63;
ctrl2_bottom = 2.62;
ctrl2_top = 13.1;

// controller 1
ctrl1_left = 69.61;
ctrl1_right = 101.52;
ctrl1_bottom = 2.62;
ctrl1_top = 13.1;

// SD card
sd_left = 16.97;
sd_right = 45.50;
sd_bottom = board_thickness; // from bottom of board
sd_top = board_thickness + 3.0; // 45.9; // from bottom of board

// cutout for serial port dongle
serial_front = 33.04;
serial_back = 52.95;
serial_top = 13.13; // from bottom of board; calipers include board
serial_bottom = 6.95; // from bottom of board; calipers include board

// power connector
wire_5v_is_proud = 2.11;
power_jack_center = [0, 75.5, -6.04 + board_thickness]; // from front, below board bottom (calipers include board)
power_jack_diameter = 6.56;

// spring-button connectors for USER1, USER2, USER3

user1_height = button_height;
user1_center = [90.22, 21.31] ; // from left, front
user1_diameter = button_diameter;

user2_height = button_height;
user2_center = [103.23, 21.31] ; // from left, front
user2_diameter = button_diameter;

user3_height = button_height;
user3_center = [116.24, 21.31] ; // from left, front
user3_diameter = button_diameter;

// LED light pipes - ?? holes into which I'll glue a piece of acrylic, which have thin plastic at case top?

power_led_center = [23.41, 75.53]; // from left, front
power_led_height = 2 + board_thickness; // height from board bottom; (calipers include board)
power_led_pipe_diameter = 2; // diameter of acrylic pipe length

debug_led_center = [18.92, 75.53]; // from left, front
debug_led_height = 2 + board_thickness; // height from board bottom; (calipers include board)
debug_led_pipe_diameter = 2; // diameter of acrylic pipe length

rgb1_led_center = [90.13, 31.32]; // from left, front
rgb1_led_height = 3.5 + board_thickness; // height from board bottom; (calipers include board)
rgb1_led_pipe_diameter = 3; // diameter of acrylic pipe length

rgb2_led_center = [103.16, 31.32]; // from left, front
rgb2_led_height = 3.5 + board_thickness; // height from board bottom; (calipers include board)
rgb2_led_pipe_diameter = 3; // diameter of acrylic pipe length

rgb3_led_center = [116.19, 31.32]; // from left, front
rgb3_led_height = 3.5 + board_thickness; // height from board bottom; (calipers include board)
rgb3_led_pipe_diameter = 3; // diameter of acrylic pipe length

boss_front_left = [6.5, 6.5, board_thickness];
boss_front_right = [board_width - 6.5, 6.5, board_thickness];
boss_rear_left = [6.5, board_depth - 6.5, board_thickness];
boss_rear_right = [board_width - 6.5, board_depth - 6.5, board_thickness];
boss_outer_diameter = 10;
boss_inner_diameter = 6; // enough for screw head
boss_screw_diameter = 3; // M3 screw
boss_screw_lip_thickness = 3; // M3 screw
boss_threaded_insert_diameter = 5.4; // may need to experiment
boss_insert_iron_depth = 8.5; // room for iron pushing threaded insert

button_pressor_diameter = 3;
button_spring_thickness = .3;
button_spring_diameter = 6;

// three horizontal vent cuts over the CPU and memory

shell_outer_lower_left_front = [
    - shell_wall_thickness - shell_side_space,
    - shell_wall_thickness - shell_side_space,
    - shell_wall_thickness - max_component_height_below_board - shell_lower_half_space
    ];

shell_outer_upper_right_rear = [
    shell_wall_thickness + shell_side_space + board_width,
    shell_wall_thickness + shell_side_space + board_depth,
    shell_wall_thickness + board_thickness + max_component_height_above_board + shell_upper_half_space
    ];

shell_inner_lower_left_front = [
    - shell_side_space,
    - shell_side_space,
    - max_component_height_below_board - shell_lower_half_space
    ];

shell_inner_upper_right_rear = [
    shell_side_space + board_width,
    shell_side_space + board_depth,
    board_thickness + max_component_height_above_board + shell_upper_half_space
    ];

shell_outer_dimensions = shell_outer_upper_right_rear - shell_outer_lower_left_front;
shell_inner_dimensions = shell_inner_upper_right_rear - shell_inner_lower_left_front;

module button_pressor_additive()
{
    translate([0, 0, button_height])
        cylinder(r = button_pressor_diameter, h = shell_outer_upper_right_rear.z - button_height, $fn=50);
}

module button_pressor_subtractive()
{
    translate([0, 0, button_height])
    difference() {
        cylinder(r = button_spring_diameter, h = shell_outer_upper_right_rear.z - button_height - button_spring_thickness, $fn=50);
        cylinder(r = button_pressor_diameter, h = shell_outer_upper_right_rear.z - button_height, $fn=50);
    }
}

module button_pressors_additive()
{
    for(center = [boot0_center, reset_center, user1_center, user2_center, user3_center])
    {
        translate(center)
        button_pressor_additive();
    }
}

module button_pressors_subtractive()
{
    for(center = [boot0_center, reset_center, user1_center, user2_center, user3_center])
    {
        translate(center)
        button_pressor_subtractive();
    }
}

module shell_outer_walls()
{
    translate(shell_outer_lower_left_front)
        cube(shell_outer_dimensions);
}

module shell_inner_walls()
{
    translate(shell_inner_lower_left_front)
        cube(shell_inner_dimensions);
}

module microusb()
{
    translate([microusb_left, board_depth - 10, 0])
        cube([microusb_right - microusb_left, 20, microusb_top]);
}

module audio_out()
{
    translate(audio_out_center)
        translate([0, -10, 0])
            rotate([-90, 0, 0])
                cylinder(r = audio_out_diameter / 2, h=20, $fn=50);
}

module audio_in()
{
    translate(audio_in_center)
        translate([0, -10, 0])
            rotate([-90, 0, 0])
                cylinder(r = audio_in_diameter / 2, h=20, $fn=50);
}

module composite_video()
{
    translate([comp_left, board_depth - 10, 0])
        cube([comp_width, 20, comp_top]);
}

module vga()
{
// VGA connector
    translate([vga_left, board_depth - 10, 0])
        cube([vga_right - vga_left, 20, vga_top]);
}

module ctrl2()
{
// controller 2
    translate([ctrl2_left, -10, 0])
        cube([ctrl2_right - ctrl2_left, 20, ctrl2_top]);
}

module ctrl1()
{
// controller 1
    translate([ctrl1_left, -10, 0])
        cube([ctrl1_right - ctrl1_left, 20, ctrl1_top]);
}

module sd_card()
{
// controller 1
    translate([sd_left, -10, 0])
        cube([sd_right - sd_left, 20, sd_top]);
}

module swd_cutout()
{
    // swd cutout
    translate([board_width - 10, debug_front, 0])
        cube([20, debug_back - debug_front, debug_thickness + board_thickness + 5]);
}

module serial_board()
{
    // serial board
    translate([-10, serial_front, 0])
        cube([20, serial_back - serial_front, serial_top]);
}

module power_jack()
{
    // wire_5v_is_proud = 2.11;
    // power_jack_diameter = 6.56;
    translate([-10, power_jack_center.y - power_jack_diameter / 2, -10])
        cube([20, power_jack_diameter * 2, 15]);
}

module boss_upper()
{
    // 1cm cylinder from board top to top of shell
    translate([0, 0, board_thickness])
    difference() {
        cylinder(r = boss_outer_diameter / 2, h = shell_outer_upper_right_rear.z - board_thickness, $fn=50);
        cylinder(r = boss_threaded_insert_diameter / 2, h = boss_insert_iron_depth, $fn=50);
    }
}

module boss_lower_additive()
{
    translate([0,0,shell_outer_lower_left_front.z])
    difference() {
        // 1cm cylinder from board bottom to bottom of shell
        cylinder(r = boss_outer_diameter / 2, h = -shell_outer_lower_left_front.z, $fn=50);
    }
}

module boss_lower_subtractive()
{
    translate([0,0,shell_outer_lower_left_front.z])
    union() {
        // cylinder subtracted across full height or whatever of screw diameter plus fudge
        cylinder(r = boss_inner_diameter / 2, h = -shell_outer_lower_left_front.z - boss_screw_lip_thickness, $fn=50);

        // cylinder subtracted from screw wall below board (1mm?) across full height size of screw head plus fudge
        cylinder(r = boss_screw_diameter / 2, h = -shell_outer_lower_left_front.z, $fn=50);
    }
}

module connector_solids()
{
    microusb();
    audio_out();
    audio_in();
    composite_video();
    vga();
    ctrl2();
    ctrl1();
    sd_card();
    serial_board();
    swd_cutout();
    power_jack();
}

module shell()
{
    difference()
    {
        shell_outer_walls();
        shell_inner_walls();
    }
}

lip_height = 1;
lip_width = shell_wall_thickness / 2;

module lip(fudge)
{
    split_lower_left_front = [
        - shell_wall_thickness / 2 - shell_side_space - fudge,
        - shell_wall_thickness / 2 - shell_side_space - fudge,
        - max_component_height_below_board - shell_lower_half_space
        ];

    split_upper_right_rear = [
        shell_wall_thickness / 2 + shell_side_space + board_width,
        shell_wall_thickness / 2 + shell_side_space + board_depth,
        board_thickness + max_component_height_above_board + shell_upper_half_space
        ];

    split_dimensions = split_upper_right_rear - split_lower_left_front;

    intersection() {
        difference() {
            translate(split_lower_left_front)
                cube(split_dimensions);
            shell_inner_walls();
        }
        translate([split_lower_left_front.x, split_lower_left_front.z, 0])
            cube([1000, 1000, lip_height]);
    }
}

module upper_bosses()
{
    translate([boss_front_left.x, boss_front_left.y, 0]) boss_upper();
    translate([boss_front_right.x, boss_front_right.y, 0]) boss_upper();
    translate([boss_rear_left.x, boss_rear_left.y, 0]) boss_upper();
    translate([boss_rear_right.x, boss_rear_right.y, 0]) boss_upper();
}

module lower_bosses_additive()
{
    translate([boss_front_left.x, boss_front_left.y, 0]) boss_lower_additive();
    translate([boss_front_right.x, boss_front_right.y, 0]) boss_lower_additive();
    translate([boss_rear_left.x, boss_rear_left.y, 0]) boss_lower_additive();
    translate([boss_rear_right.x, boss_rear_right.y, 0]) boss_lower_additive();
}

module lower_bosses_subtractive()
{
    translate([boss_front_left.x, boss_front_left.y, 0]) boss_lower_subtractive();
    translate([boss_front_right.x, boss_front_right.y, 0]) boss_lower_subtractive();
    translate([boss_rear_left.x, boss_rear_left.y, 0]) boss_lower_subtractive();
    translate([boss_rear_right.x, boss_rear_right.y, 0]) boss_lower_subtractive();
}


module top_half()
{
    difference() {
        difference() {
            intersection() {
                shell();
                translate([-500, -500, 0]) cube([1000, 1000, 1000]);
            }
            lip(0);
        }
        union() {
            connector_solids();
            button_pressors_subtractive();
        }
    }
    upper_bosses();
    button_pressors_additive();
}

module bottom_half()
{
    difference() {
        union() {
            intersection() {
                shell();
                translate([-500, -500, -1000]) cube([1000, 1000, 1000]);
            }
            lip(-.2);
            lower_bosses_additive();
        }
        union() {
            connector_solids();
            lower_bosses_subtractive();
        }
    }
}

translate([0, 0, shell_outer_upper_right_rear.z]) rotate([180, 0, 0]) top_half();
translate([0, 10, -shell_outer_lower_left_front.z]) bottom_half();
