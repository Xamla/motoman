uint8 intp_type      # Type of interpolation(MOV?)
uint8 intp_kind      # Kind of interpolation(?MOV)
uint8 speed_type     # Type of speed
int32 speed_value    # Speed value(0.01[%] or 0.1[mm/sec], -1:not set)
JobPosData[] posData # JOB position data(Robot, Base, Station)