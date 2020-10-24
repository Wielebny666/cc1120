set(SOURCES "cc112x_hal.c" "cc112x.c")

idf_component_register(SRCS "${SOURCES}"
					INCLUDE_DIRS .
					REQUIRES 	foreach 
								spi_master_ext)

