# MainController

Firmware for the Elecrow 7-inch HMI connected through the USB-SERIAL CH340.

## Required Arduino board setting

- Board: ESP32S3 Dev Module
- Flash: 4 MB
- PSRAM: OPI PSRAM
- Display resolution: 800 x 480

The RGB framebuffer requires the 8 MB OPI PSRAM. A build made with PSRAM
disabled can compile and upload successfully but will restart when the display
tries to draw its first screen.

## Interface draft

`HmiWorkflowDraft.h` contains the replaceable screen workflow. The sensor,
speaker, touch and ESP-NOW hardware integration remains in `MainController.ino`.

The draft includes visible simulation buttons so the full navigation can be
reviewed without presenting a face, badge or electrical test result. These
buttons are temporary and should be removed from the production interface.
