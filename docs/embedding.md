# Embedding as a C++ library

`GearsonicInference` is the system facade — the single owner of module
wiring and startup order. Link the `gearsonic_inference` CMake target and
drive the lifecycle from your own process:

```cmake
add_subdirectory(kist-gearsonic-inference)
target_link_libraries(your_app PRIVATE gearsonic_inference)
```

```cpp
#include "system/gearsonic_inference.hpp"
#include "motion/input_handler.hpp"

auto& gearsonic_inf = kist::GearsonicInference::instance();
gearsonic_inf.install_signal_handlers();          // or call gearsonic_inf.request_quit() from your own handler

if (!gearsonic_inf.start("config/config.yaml"))   // THE ROBOT MOVES: 3s ramp, then policy control
    return 1;

// external navigation (optional): body-frame velocity, ~20Hz.
// zeros = stop, going silent = fallback to manual. Joystick always wins.
kist::InputHandler::instance().nav_buf.SetData({vx, vy, vyaw});

// ... your application runs here (keep the process alive) ...

gearsonic_inf.stop();                             // publishes damping — call on every exit path
```
