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
power_jack_center = [0, 76.5, -6.04 + board_thickness]; // from front, below board bottom (calipers include board)
power_jack_diameter = 7.06;

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

// LED light pipes

led_pipe_inner_diameter = 3;
led_pipe_outer_diameter = led_pipe_inner_diameter + 2;
led_pipe_recess_diameter = 4.3;
led_pipe_recess_depth = .3;

power_led_center = [23.41, 75.53]; // from left, front
power_led_height = 2 + board_thickness; // height from board bottom; (calipers include board)

debug_led_center = [18.92, 75.53]; // from left, front
debug_led_height = 2 + board_thickness; // height from board bottom; (calipers include board)

rgb1_led_center = [90.13, 31.32]; // from left, front
rgb1_led_height = 3.5 + board_thickness; // height from board bottom; (calipers include board)

rgb2_led_center = [103.16, 31.32]; // from left, front
rgb2_led_height = 3.5 + board_thickness; // height from board bottom; (calipers include board)

rgb3_led_center = [116.19, 31.32]; // from left, front
rgb3_led_height = 3.5 + board_thickness; // height from board bottom; (calipers include board)

boss_front_left = [6.5, 6.5, board_thickness];
boss_front_right = [board_width - 6.5, 6.5, board_thickness];
boss_rear_left = [6.5, board_depth - 6.5, board_thickness];
boss_rear_right = [board_width - 6.5, board_depth - 6.5, board_thickness];
boss_outer_diameter = 10;
boss_inner_diameter = 6; // enough for screw head
boss_screw_diameter = 3.5; // M3 screw
boss_screw_lip_thickness = 3; // M3 screw
boss_threaded_insert_diameter = 5.3; // may need to experiment
boss_insert_iron_depth = 8.5; // room for iron pushing threaded insert

button_pressor_diameter = 6;
button_insert_shaft_diameter = 1;
button_spring_thickness = .3;
button_spring_diameter = 12;

rubberfoot_diameter = 10;
rubberfoot_indent = 1;

rubberfoot_centers = [
    [17, 17],
    [board_width - 17, 17],
    [17, board_depth - 17],
    [board_width - 17, board_depth - 17],
];

cpumem_left = 52.87;
cpumem_right = 104.03;
cpumem_front = 45.82;
cpumem_rear = 70.35;

// each slot is 2mm gap, 3mm of recessed channel, then 5mm of case
cpumem_slot_gap = 1;
cpumem_slot_recessed = 2;
cpumem_slot_recessed_overhang = 2;
cpumem_slot_recessed_thickness = 2;
cpumem_slot_case = 2;
cpumem_slot_depth = cpumem_slot_gap + cpumem_slot_recessed + cpumem_slot_case;
cpumem_slot_count = floor((cpumem_rear - cpumem_front) / cpumem_slot_depth);

logo_plate_left = 5;
logo_plate_right = logo_plate_left + 30;
logo_plate_front = 5;
logo_plate_rear = logo_plate_front + 10;
logo_plate_thickness = .5;

module logo_plate_subtractive()
{
    translate([logo_plate_left, logo_plate_front, shell_outer_right_rear_upper[2] - logo_plate_thickness])
    cube([logo_plate_right - logo_plate_left, logo_plate_rear - logo_plate_front, logo_plate_thickness + 1]);
}

module cpumem_slots_additive()
{
    for(slot = [0 : cpumem_slot_count - 1]) {
        translate([cpumem_left - cpumem_slot_recessed_overhang, cpumem_front + slot * cpumem_slot_depth + cpumem_slot_gap, shell_outer_right_rear_upper[2] - shell_wall_thickness - cpumem_slot_recessed_thickness])
            cube([cpumem_right - cpumem_left + 2 * cpumem_slot_recessed_overhang, cpumem_slot_recessed + cpumem_slot_case, cpumem_slot_recessed_thickness]);
    }
}

module cpumem_slots_subtractive()
{
    for(slot = [0 : cpumem_slot_count - 1]) {
        translate([cpumem_left, cpumem_front + slot * cpumem_slot_depth, shell_outer_right_rear_upper[2] - shell_wall_thickness])
            cube([cpumem_right - cpumem_left, cpumem_slot_gap + cpumem_slot_recessed, shell_wall_thickness]);
    }
}

// three horizontal vent cuts over the CPU and memory

shell_outer_left_front_lower = [
    - shell_wall_thickness - shell_side_space,
    - shell_wall_thickness - shell_side_space,
    - shell_wall_thickness - max_component_height_below_board - shell_lower_half_space
    ];

shell_outer_right_rear_upper = [
    shell_wall_thickness + shell_side_space + board_width,
    shell_wall_thickness + shell_side_space + board_depth,
    shell_wall_thickness + board_thickness + max_component_height_above_board + shell_upper_half_space
    ];

shell_inner_left_front_lower = [
    - shell_side_space,
    - shell_side_space,
    - max_component_height_below_board - shell_lower_half_space
    ];

shell_inner_right_rear_upper = [
    shell_side_space + board_width,
    shell_side_space + board_depth,
    board_thickness + max_component_height_above_board + shell_upper_half_space
    ];

module rubberfoot_subtractive()
{
    for(center = rubberfoot_centers) {
        translate([center[0], center[1], shell_outer_left_front_lower[2]])
            cylinder(rubberfoot_indent, rubberfoot_diameter / 2, rubberfoot_diameter / 2);
    }
}


shell_outer_dimensions = shell_outer_right_rear_upper - shell_outer_left_front_lower;
shell_inner_dimensions = shell_inner_right_rear_upper - shell_inner_left_front_lower;

module button_pressor_additive()
{
    translate([0, 0, button_height + .2])
        cylinder(r = button_pressor_diameter / 2, h = shell_outer_right_rear_upper.z - button_height - .2, $fn=50);
}

module button_pressor_subtractive()
{
    translate([0, 0, button_height])
    difference() {
        cylinder(r = button_spring_diameter / 2, h = shell_outer_right_rear_upper.z - button_height - button_spring_thickness, $fn=50);
        cylinder(r = button_pressor_diameter / 2, h = shell_outer_right_rear_upper.z - button_height, $fn=50);
    }
    translate([0, 0, -2.5])
    cylinder(r = button_insert_shaft_diameter / 2, h = shell_outer_right_rear_upper.z + 5);
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

module led_pipe_additive(center, height)
{
    translate(center + [0, 0, height])
        cylinder(r = led_pipe_outer_diameter / 2, h = shell_outer_right_rear_upper.z - board_thickness, $fn=50);
}

module led_pipe_subtractive(center, height)
{
    union() {
        translate(center + [0, 0, -.1])
            cylinder(r = led_pipe_inner_diameter / 2, h = shell_outer_right_rear_upper.z + .2, $fn=50);
        translate([center[0], center[1], shell_outer_right_rear_upper[2] - led_pipe_recess_depth])
            cylinder(r = led_pipe_recess_diameter / 2, h = led_pipe_recess_depth, $fn=50);
    }
}

leds = [
    [power_led_center, power_led_height],
    [debug_led_center, debug_led_height],
    [rgb1_led_center, rgb1_led_height],
    [rgb2_led_center, rgb2_led_height],
    [rgb3_led_center, rgb3_led_height]
    ];

module led_pipes_additive()
{
    for(led = leds) {
        led_pipe_additive(led[0], led[1]);
    }
}

module led_pipes_subtractive()
{
    for(led = leds) {
        led_pipe_subtractive(led[0], led[1]);
    }
}

// Rounded cube from https://danielupshaw.com/openscad-rounded-corners/

// Higher definition curves
$fs = 0.1;

module roundedcube(size = [1, 1, 1], center = false, radius = 0.5, apply_to = "all")
{
    // If single value, convert to [x, y, z] vector
    size = (size[0] == undef) ? [size, size, size] : size;

    translate_min = radius;
    translate_xmax = size[0] - radius;
    translate_ymax = size[1] - radius;
    translate_zmax = size[2] - radius;

    diameter = radius * 2;

    module build_point(type = "sphere", rotate = [0, 0, 0]) {
        if (type == "sphere") {
            sphere(r = radius);
        } else if (type == "cylinder") {
            rotate(a = rotate)
            cylinder(h = diameter, r = radius, center = true);
        }
    }

    obj_translate = (center == false) ?
        [0, 0, 0] : [
            -(size[0] / 2),
            -(size[1] / 2),
            -(size[2] / 2)
        ];

    translate(v = obj_translate) {
        hull() {
            for (translate_x = [translate_min, translate_xmax]) {
                x_at = (translate_x == translate_min) ? "min" : "max";
                for (translate_y = [translate_min, translate_ymax]) {
                    y_at = (translate_y == translate_min) ? "min" : "max";
                    for (translate_z = [translate_min, translate_zmax]) {
                        z_at = (translate_z == translate_min) ? "min" : "max";

                        translate(v = [translate_x, translate_y, translate_z])
                        if (
                            (apply_to == "all") ||
                            (apply_to == "xmin" && x_at == "min") || (apply_to == "xmax" && x_at == "max") ||
                            (apply_to == "ymin" && y_at == "min") || (apply_to == "ymax" && y_at == "max") ||
                            (apply_to == "zmin" && z_at == "min") || (apply_to == "zmax" && z_at == "max")
                        ) {
                            build_point("sphere");
                        } else {
                            rotate =
                                (apply_to == "xmin" || apply_to == "xmax" || apply_to == "x") ? [0, 90, 0] : (
                                (apply_to == "ymin" || apply_to == "ymax" || apply_to == "y") ? [90, 90, 0] :
                                [0, 0, 0]
                            );
                            build_point("cylinder", rotate);
                        }
                    }
                }
            }
        }
    }
}

module shell_outer_walls()
{
    translate(shell_outer_left_front_lower)
        cube(shell_outer_dimensions); // , radius=2);
}

module shell_inner_walls()
{
    translate(shell_inner_left_front_lower)
        cube(shell_inner_dimensions); // , radius=2);
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
    translate([-10, power_jack_center.y - power_jack_diameter / 2, -10])
        cube([20, power_jack_diameter * 2, 15]);
}

module boss_upper()
{
    // 1cm cylinder from board top to top of shell
    translate([0, 0, board_thickness])
    difference() {
        cylinder(r = boss_outer_diameter / 2, h = shell_outer_right_rear_upper.z - board_thickness, $fn=50);
        cylinder(r = boss_threaded_insert_diameter / 2, h = boss_insert_iron_depth, $fn=50);
    }
}

module boss_lower_additive()
{
    translate([0,0,shell_outer_left_front_lower.z])
    difference() {
        // 1cm cylinder from board bottom to bottom of shell
        cylinder(r = boss_outer_diameter / 2, h = -shell_outer_left_front_lower.z, $fn=50);
    }
}

module boss_lower_subtractive()
{
    translate([0,0,shell_outer_left_front_lower.z])
    union() {
        // cylinder subtracted across full height or whatever of screw diameter plus fudge
        cylinder(r = boss_inner_diameter / 2, h = -shell_outer_left_front_lower.z - boss_screw_lip_thickness, $fn=50);

        // cylinder subtracted from screw wall below board (1mm?) across full height size of screw head plus fudge
        cylinder(r = boss_screw_diameter / 2, h = -shell_outer_left_front_lower.z, $fn=50);
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
    split_left_front_lower = [
        - shell_wall_thickness / 2 - shell_side_space - fudge,
        - shell_wall_thickness / 2 - shell_side_space - fudge,
        - max_component_height_below_board - shell_lower_half_space
        ];

    split_right_rear_upper = [
        shell_wall_thickness / 2 + shell_side_space + board_width,
        shell_wall_thickness / 2 + shell_side_space + board_depth,
        board_thickness + max_component_height_above_board + shell_upper_half_space
        ];

    split_dimensions = split_right_rear_upper - split_left_front_lower;

    intersection() {
        difference() {
            translate(split_left_front_lower)
                cube(split_dimensions);
            shell_inner_walls();
        }
        translate([split_left_front_lower.x, split_left_front_lower.z, 0])
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
                union() {
                    shell();
                    upper_bosses();
                    button_pressors_additive();
                    led_pipes_additive();
                    cpumem_slots_additive();
                }
                translate([-500, -500, 0]) cube([1000, 1000, 1000]);
            }
            lip(0);
        }
        union() {
            connector_solids();
            button_pressors_subtractive();
            led_pipes_subtractive();
            cpumem_slots_subtractive();
            logo_plate_subtractive();
        }
    }
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
            rubberfoot_subtractive();
        }
    }
}

// top half of case
translate([0, 0, shell_outer_right_rear_upper.z]) rotate([180, 0, 0]) top_half();

// bottom half of case
translate([0, 10, -shell_outer_left_front_lower.z]) bottom_half();

// button top
// cylinder(h = .5, r = button_pressor_diameter / 2);
// cylinder(r = button_insert_shaft_diameter / 2 - .05, h = 5);

// led light pipe insert
// cylinder(h = led_pipe_recess_depth, r = led_pipe_recess_diameter / 2 - .05);
