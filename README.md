Ever since I was very young, I've wanted to output video from a circuit I've built myself.  When I was a teenager, I bought a "TV modulator" module from Radio Shack.  With a minimum set of other components (mostly just power, if I remember correctly), it would turn composite NTSC video into broadcast VHF channel 3 or 4.

In the Alice 2, we built a very rudimentary NTSC circuit.  An UV-erasable EPROM provided the sync signals and a timer clocked out one bit pixels for video data.  It worked, but not very well.  17 years later, the circuit had degraded so that video output was badly broken and my attempts to redesign and upgrade the circuit fizzled.  We decided to abandon the old design and build the Alice 3 instead.

In the Alice 3, we used a dedicated Propeller to output a textport to VGA frequencies from a Z80, and used an ARM to control peripherals including PS/2 keyboard and microSD.  But ARM microcontrollers these days have pretty advanced functionality and high clock rates.  NTSC is an old standard now, first supplanted by high clock rates and color resolutions in VGA, and later pure digital signals over HDMI and DisplayPort.  Still, the idea of building a complete "modern" "TV typewriter" with just an ARM and minimum parts seemed cool to me, so I built one.  I describe the design, capabilities, and limitations here.

DAC

color

framebuffer implementation


