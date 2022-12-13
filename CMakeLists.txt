add_executable(spi_switch
        spi_switch.c
        )

# Pull in basic dependencies
target_link_libraries(spi_switch pico_stdlib hardware_spi hardware_adc)

# create map/bin/hex file etc.
pico_add_extra_outputs(spi_switch)

# add url via pico_set_program_url
example_auto_set_url(spi_switch)
