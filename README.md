# Wazuka-Crossing
Miniature of a Japanese-style level crossing

## Full Version (with level crossing barriers)

**Under Development**

## Simple Version (without level crossing barriers)
[![Level Crossing Warning Sound Generator](https://img.youtube.com/vi/Fo-AsSxr2Go/0.jpg)](https://www.youtube.com/watch?v=Fo-AsSxr2Go)

### MCU
- PIC16F15213 PDIP / SOIC / DFN

|Pin| Name         | Function | Target    |
|---|--------------|----------|-----------|
| 1 | VDD          | VDD      |           |
| 2 | RA5          | DO       | Lamp 1    |
| 3 | RA4          | DO       | Lamp 2    |
| 4 | MCLR         | MCLR     |           |
| 5 | PWM3         | PWM      | Speaker 1 |
| 6 | ICSPCLK/PWM4 | ICSP/PWM | Speaker 2 |
| 7 | ICSPDAT      | ICSP/DI  | Switch    |
| 8 | VSS          | VSS      |           |

## Timers
| Freq.  | Timer             | Purpose                           |
|-------:|:------------------|-----------------------------------|
| 20 kHz | Timer2            | Sound PWM (PWM3)                  |
| 156 Hz | Timer2 / 128      | Lamp PFM (S/W)                    |
| ~30 Hz | Timer2 / 4 / ~150 | Sound Modulation                  |
| 20 Hz  | Timer0            | Train Detection, Button           |
| 1.7 Hz | Timer0 / 12       | Lamp Flicker                      |

## Notes
The PICkit3 does not officially support the PIC16F152 family.
You can program PIC16F152 devices by PICkit3 with
[PICkitminus](http://kair.us/projects/pickitminus/).
