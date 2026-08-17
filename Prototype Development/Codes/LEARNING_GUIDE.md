# ESD station firmware learning guide

This guide explains where to begin. Read one controller at a time; do not try to
understand every library call during the first pass.

## 1. The information shared between controllers

Start with `EspNowProtocol.h`. `EsdNowMessage` is the common packet sent through
ESP-NOW. The MainController receives tester fields from SecondaryController and
face fields from FacialController.

## 2. SecondaryController

Read `SecondaryController.ino` in this order:

1. Pin and timing constants.
2. `readInputFlags()` for the four HZR-171 signals.
3. The debounce section inside `loop()`.
4. `sendStatus()` for the packet sent to MainController.
5. The GM67 functions for a one-shot badge session.

Data path:

`HZR-171 GPIOs -> debounce -> stableFlags -> EsdNowMessage -> ESP-NOW`

## 3. MainController

Read `MainController.ino` from `setup()` and `loop()` first. Then study
`processEspNowMessage()` and `processWorkflowActions()`.

Data path:

`remote packet -> MainController -> HmiWorkflowDraft -> screen/action`

## 4. HMI workflow

`HmiWorkflowDraft.h` is a state machine. The `Screen` enum lists all possible
states. `changeScreen()` changes state, and `draw()` selects the matching page.

The ESD test follows this sequence:

`BP HIGH -> start timer -> wait 2 s -> copy latest signals -> final result`

The live `latest*` variables can continue changing. The `final*` variables are a
snapshot and are the only values saved in the report.

## 5. FacialController

Read `FacialController.ino` first. It initializes the camera and radio and sends
recognition results. Then read only these functions in `FacialWebServer.cpp`:

1. `autonomous_face_task()`
2. `startAutonomousFaceRecognition()`

The other web-server functions are advanced supporting code and are not required
to understand the current autonomous prototype.

## Questions to ask while reading

For each function, identify:

1. Who calls it?
2. What inputs does it receive?
3. What state does it change?
4. What output or message does it produce?
5. What happens if the hardware or communication fails?

