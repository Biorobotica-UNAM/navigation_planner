uint8 SOURCE_NONE=0
uint8 SOURCE_ACTION=1
uint8 SOURCE_TOPIC=2

# Active status codes match Navigate feedback.
uint8 RESOLVING=0
uint8 CONFIGURING=1
uint8 PLANNING=2
uint8 FOLLOWING=3
uint8 RECOVERING=4
uint8 VERIFYING=5
uint8 CANCELLING=6
uint8 IDLE=7
uint8 FINISHED=8

std_msgs/Header header
uint64 goal_id
uint8 source
uint8 status_code
string status_message
float32 distance_remaining_m
uint16 total_recoveries
bool result_valid
uint8 result_code
string reason
string result_message
float32 position_error_m
float32 orientation_error_rad
bool faulted
