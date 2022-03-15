file(REMOVE_RECURSE
  "CMakeFiles/generate_message"
  "message/message_gnss.pb.cc"
  "message/message_gnss.pb.h"
  "message/message_header.pb.cc"
  "message/message_header.pb.h"
  "message/message_imu.pb.cc"
  "message/message_imu.pb.h"
  "message/message_map.pb.cc"
  "message/message_map.pb.h"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/generate_message.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
