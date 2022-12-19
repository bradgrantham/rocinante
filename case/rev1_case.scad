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

pushbutton_height = 8.72; // from bottom of board
pushbutton_diameter = 3.37;

// spring-button connectors for BOOT0, RESET
boot0_height = pushbutton_height;
boot0_center = [32.82, 78.39] ; // from left, from front
boot0_diameter = pushbutton_diameter;

reset_height = pushbutton_height;
reset_center = [32.82, 67.51] ; // from left, from front
reset_diameter = pushbutton_diameter;

// audio out connector
audio_out_center = [44.68, 8.47]; // from left, from board top
audio_out_diameter = 6;

// audio in connector
audio_in_center = [66.59, 8.47]; // from left, from board top
audio_in_diameter = 6;

// composite connector
comp_left = 81.44; // from left
comp_width = 10;
comp_bottom = 0 ; // from bottom of board
comp_top = 13.07; // from bottom of board

// VGA connector
vga_left = 111;
vga_right = 127.52;
vga_bottom = 3.81;
vga_top = 12.20;

// debug cable connector cutout

debug_front = 58.12; // from front of board
debug_back = 71.40; // from front of board
debug_thickness = .63; // need at least this much cut out for cable

// a little space for the PS/2 connector sticking out but no reason to cut out for the connector
ps2_is_proud = .87;

// controller 2
ctrl2_left = 107.05;
ctrl2_right = 136.63;
ctrl2_bottom = 2.62;
ctrl2_top = 13.1;

// controller 1
ctrl1_left = 70.61;
ctrl1_right = 100.52;
ctrl1_bottom = 2.62;
ctrl1_top = 13.1;

// SD card
sd_left = 18.97;
sd_right = 43.50;
sd_bottom = board_thickness; // from bottom of board
sd_top = 45.9; // from bottom of board

// cutout for serial port dongle
serial_front = 34.54;
serial_back = 51.95;
serial_top = 13.13; // from bottom of board; calipers include board
serial_bottom = 6.95; // from bottom of board; calipers include board

// power connector
wire_5v_is_proud = 2.11;
power_jack_center = [83.52, -6.04 + board_thickness]; // from front, below board bottom (calipers include board)
power_jack_diameter = 6.56;

// spring-button connectors for USER1, USER2, USER3

user1_height = pushbutton_height;
user1_center = [90.22, 21.31] ; // from left, front
user1_diameter = pushbutton_diameter;

user2_height = pushbutton_height;
user2_center = [103.23, 21.31] ; // from left, front
user2_diameter = pushbutton_diameter;

user3_height = pushbutton_height;
user3_center = [116.24, 21.31] ; // from left, front
user3_diameter = pushbutton_diameter;

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

module shell()
{
    difference() {
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
    echo(fudge);

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

module top_half()
{
    difference() {
        intersection() {
            shell();
            translate([-500, -500, 0]) cube([1000, 1000, 1000]);
        }
        lip(0);
    }
}

module bottom_half()
{
    intersection() {
        shell();
        translate([-500, -500, -1000]) cube([1000, 1000, 1000]);
    }
    lip(-.2);
}

translate([0, 0, shell_outer_upper_right_rear.z]) rotate([180, 0, 0]) top_half();
translate([0, 10, -shell_outer_lower_left_front.z]) bottom_half();
