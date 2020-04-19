# README WORK IN PROGRESS

Ever since I was very young, I've wanted to output video from a circuit I've built myself.  When I was a teenager, I bought a "TV modulator" module from Radio Shack.  With a minimum set of other components (mostly just power, if I remember correctly), it would turn composite NTSC video into broadcast VHF channel 3 or 4.

In the Alice 2, we built a very rudimentary NTSC circuit.  An UV-erasable EPROM provided the sync signals and a timer clocked out one bit pixels for video data.  It worked, but not very well.  17 years later, the circuit had degraded so that video output was badly broken and my attempts to redesign and upgrade the circuit fizzled.  We decided to abandon the old design and build the Alice 3 instead.

PIC OF ALICE 2 VIDEO BOARD AND IMAGE

In the Alice 3, we used a dedicated Propeller to output a textport to VGA frequencies from a Z80, and used an ARM to control peripherals including PS/2 keyboard and microSD.

PIC OF ALICE 3

But ARM microcontrollers these days have pretty advanced functionality and high clock rates.  NTSC is an old standard now, first supplanted by high clock rates and color resolutions in VGA, and later pure digital signals over HDMI and DisplayPort.  Still, the idea of building a complete "modern" "TV typewriter" with just an ARM and minimum parts seemed cool to me, so I built one.  I describe the design, capabilities, and limitations here.

# Color

Every NTSC scanline has a “colorburst” in the off-screen part of the line, at 3.579545 MHz.  This burst has the same *average* amplitude as the off-screen blank part of the line, and was much higher frequency than the usual pixel frequency, so B/W sets would just ignore it.

A color scanline was separated into luminance and chroma before transmission. A 3.579545MHz signal encoded the chroma with its phase (the hue; e.g. blue is 347 degrees, red is 103, green is 241) and its amplitude (saturation; 0 is gray, 1 is full color).  The luminance signal (0 to 1) and chroma (-1 to +1 scaled by the saturation) were added together for broadcast or composite cables.

Color TVs know to separate out the high-frequency component and turn that back into hue, saturation, and value (luminance or brightness).  B/W sets would not separate the high-frequency color signal so would just display the average signal so it would just show up as grayscale.

I think there’s some subtlety there to broadcast and where the signals lie in spectrum allocated to the channel.  I don’t understand radio, so I’ll just wave my hands.

Here’s a picture I’ve turned into NTSC video.  The green lines happen because I get a glitch sometimes in which causes the pixels are offset by some fraction of a 3.579545MHz wave, so the colors are all rotated around the color wheel.

PICTURE

# DAC

8 pins from a microcontroller are connected through resistors to form a (really bad) analog signal which I make slightly better with capacitors.

# Signal generation

I’m pretty sure a framebuffer from the 70's and 80's would turn RGB into YUV (“YIQ” for NTSC) and use logic to indicate when horizontal and digital sync signals should be.  Those would feed an analog circuit that would output a much higher quality analog signal.

I just drive the entire signal with DMA through the DAC

