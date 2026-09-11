#pragma once

#include <string>
#include <thread>

namespace kist {

// Mirrors everything the process writes to stdout/stderr (std::cout,
// printf, SDK C output alike) into one file, while it still reaches the
// terminal. Works at the file-descriptor level: fds 1 and 2 are pointed
// at a pipe and a reader thread copies the stream to the original tty
// and the file. Lines land in the file as they are written, so a run
// that dies or whose terminal is closed keeps its log up to that point.
//
// One file per run, overwritten: the latest run is what you look at.
class ConsoleTee {
public:
    static ConsoleTee& instance();

    // Creates the parent directory as needed. On failure logs a warning
    // and leaves the console untouched (returns false).
    bool start(const std::string& path);
    // Restores fds 1/2 and drains the pipe. Safe to call when not started.
    void stop();

private:
    ConsoleTee() = default;
    void pump();

    int  file_fd_{-1};
    int  saved_out_{-1};
    int  saved_err_{-1};
    int  pipe_r_{-1};
    int  pipe_w_{-1};
    bool active_{false};
    std::thread pump_thread_;
};

} // namespace kist
