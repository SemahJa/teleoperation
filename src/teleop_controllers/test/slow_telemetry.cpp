// Test-only publisher stall: reproduce a slow middleware call without robots.
#include <dlfcn.h>
#include <rcl/publisher.h>

#include <chrono>
#include <cstring>
#include <thread>

extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* publisher, const void* message,
                                 rmw_publisher_allocation_t* allocation) {
  using Publish = decltype(&rcl_publish);
  static const auto publish = reinterpret_cast<Publish>(dlsym(RTLD_NEXT, "rcl_publish"));
  const auto* topic = rcl_publisher_get_topic_name(publisher);
  if (topic && std::strstr(topic, "/robot_state"))
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
  return publish(publisher, message, allocation);
}
