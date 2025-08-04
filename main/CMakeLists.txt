idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES mdb_slave
                    PRIV_REQUIRES nvs_flash)
