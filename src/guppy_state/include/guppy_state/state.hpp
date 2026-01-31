#ifndef GUPPY_STATE
#define GUPPY_STATE

#include <cstdint>

namespace guppy_state {
    enum class State : std::uint8_t {
        INITIAL,    
        HOLDING,
        NAV,
        TASK,
        TELEOP,
        DISABLED,
        FAULT
    };
}

#endif // GUPPY_STATE