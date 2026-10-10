## IO Device status
builtin_interfaces/Time time# time the message was created
intera_core_msgs/IOComponentStatus device# device status data
intera_core_msgs/IODataStatus[] ports# Ports status
intera_core_msgs/IODataStatus[] signals# Signals status
builtin_interfaces/Time[] commands# recent command timestamps, for syncing
string[] responses# recent command responses

