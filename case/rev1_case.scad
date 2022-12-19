// 0, 0, 0 is left, front corner of *bottom* of board.
// therefore board is resting on YX plane

board_width = 150;
board_depth = 100;
board_thickness = 1.6;

max_component_height_above_board = 16.30 - board_thickness; // populated ESP-01.  Calipers include board
max_component_height_below_board = 10.74 - board_thickness; // Power barrel.  Calipers include board

box_upper_half_space = 1; // space between components and box top
box_bottom_half_space = 1; // space between components and box bottom
box_side_space = 1; // space between boards and box sides
box_wall_thickness = 1;

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

box_upper_half_dimensions = [
    board_width + box_wall_thickness * 2 + box_side_space * 2,
    board_depth + box_wall_thickness * 2 + box_side_space * 2,
    board_thickness + max_component_height_above_board + box_upper_half_space + box_wall_thickness
    ];

empty_upper_half_dimensions = [
    board_width + box_side_space * 2,
    board_depth + box_side_space * 2,
    board_thickness + max_component_height_above_board + box_upper_half_space
    ];

module upper_box_half() {
    difference() {
        cube(box_upper_half_dimensions);
        translate([box_wall_thickness, box_wall_thickness, 0]) cube(empty_upper_half_dimensions);
    }
};

// subtract all negative components of parts
// add back all positive components of parts

upper_box_half();

// halves of enclosure for printing
    // bottom is
        // shell minus everything above YX plane, bottom of board
        // plus lip minus fudge
    // top is shell minus everything below YX plane
        // minus lip
