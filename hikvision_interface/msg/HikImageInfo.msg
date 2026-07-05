# header
std_msgs/Header header
# exposure timestamp in system/PTP time domain when valid
builtin_interfaces/Time dev_stamp
uint32 frame_num
# acqusistion
float32 gain
float32 exposure
# white balance
uint32 red
uint32 green
uint32 blue

# trigger/exposure matching diagnostics
bool matched_trigger
bool ptp_locked
bool ptp_time_valid
uint64 trigger_seq
uint32 trigger_index
builtin_interfaces/Time t_app_issue
builtin_interfaces/Time t_drv_before
builtin_interfaces/Time t_drv_after
int64 delta_ctrl_ns
int64 delta_drv_ns

# raw timestamp diagnostics
uint64 dev_timestamp_ticks
uint64 timestamp_tick_frequency
uint32 chunk_second_count
uint32 chunk_cycle_count
uint32 chunk_cycle_offset
