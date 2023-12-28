Wiring diagrams and schematics can be found in the PDF files in this folder.
There are also [Fritzing](https://fritzing.org/) project files (*.fzz) if you want to modify them.

> [!NOTE] 
> The files with `5V_` prefix show how a [logic level shifter](https://en.wikipedia.org/wiki/Level_shifter) can be used to use a higher voltage in the data signal - using the 3.3V directly from the Pico is out of spec for most WS281x chips, but usually works fine. If you notice glitching when using longer wires for the data line, you should consider using the logic shifter.

> [!TIP]
> If your lights use 5V power, you can skip the DC/DC converter

Used parts (non affiliate links) confirmed to be working with WS2811 lights:
- Adjustable DC/DC converter: https://www.amazon.pl/dp/B07DK64L2J
- Logic shifter (optional): https://www.amazon.pl/dp/B07NNRS8FS