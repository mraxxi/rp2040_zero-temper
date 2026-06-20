
setup:
- rp2040 zero from waveshare
- platformio in vscode

------------------------------

project files:
- main.cpp (entry point, pid calculation, general flow)
- ssr_sensors.cpp (sensors logic, ssr function[time proportional])
- enc_display.cpp (gui ui/ux, encoder logic here?)
- tuning.cpp (tuning logic, callable at user request)
headers:
- main.h (declarations, unified)
- config.h (pin mapping)

------------------------------

pin mapping: gpio_num/function
- 0 / display_data
- 1 / display_cs
- 2 / display_clk
- 3 / enc_b
- 4 / enc_a
- 5 / enc_button
- 6 / beeper
- 7 / (reserved)
- 8 / ssr_bottom
- 9 / ssr_top
- 10 / sensors_cs_1 (max6675)
- 11 / sensors_cs_2 (max6675)
- 12 / sensors_cs_3 (max31855)
- 13 / sensors_cs_4 (max31855)
- 14 / sensors_clk
- 28 / sensors_data

------------------------------

general hardware looks:
- 2 max31855 for pcb probe, one at the heating zone, one for edge temp.
- 2 max6675 for internal heater temp logging, nice to have (diagnosis, etc), not mission critical
- 2 heating zones (top/botom) each controlled by an ssr with time proportion scheme
- 1 encoder with enc_button
- 1 3v3 complatible input
- 1 U8G2_ST7920_128X64_F_SW_SPI screen display

------------------------------

general operation :
- measure all temp_data at some interval/tick (sync with pid update period?)
- use max31855 zone_temp for top heater pid calculation
- use max31855 edge_temp for bottom heater pid temp regulation
- use both max6675 heater_temp only for warning

- pool user input from encoder
- display target/current temperature, status, menu, etc to lcd

------------------------------

display:
- welcome screen splash
- main menu (display menus, internal_temps at bottom)
    - preheat menu
        - config preheat (time, target, options)
        - run preheat
            - preheat running dashboard (displays set/current temp, time since start, quit preheat, etc)
        - exit preheat menu (back to main)
    - bga rework menu
        - config bga (max_temp, profiles, warning trigger, top heating delay after preheat)
        - profile selector (display selected)
            - profile_1
            - profile_2
            - profile_n
        - run bga rework
            - bga rework mode runnign dashboard (top/heater set temp, time, quit, etc)
        - exit bga rework menu (back to main)
    - configs menu
        - beeper configs
        - enable/disable internal heater sensor logging
        - profile viewer/editor/import/export to sdcard (planned for future use)
        - all other configs
        - exit (back to main)

------------------------------

