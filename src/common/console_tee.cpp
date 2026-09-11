#include "common/console_tee.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>

namespace kist {

ConsoleTee& ConsoleTee::instance() {
    static ConsoleTee inst;
    return inst;
}

static bool write_all(int fd, const char* p, size_t n) {
    while (n > 0) {
        ssize_t w = ::write(fd, p, n);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += w; n -= static_cast<size_t>(w);
    }
    return true;
}

bool ConsoleTee::start(const std::string& path) {
    if (active_) return true;

    std::error_code ec;
    auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent, ec);

    file_fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (file_fd_ < 0) {
        std::cerr << "[ConsoleTee] cannot open " << path << ": " << std::strerror(errno)
                  << " — console not mirrored\n";
        return false;
    }
    int pfd[2];
    if (::pipe(pfd) != 0) {
        std::cerr << "[ConsoleTee] pipe failed: " << std::strerror(errno) << "\n";
        ::close(file_fd_); file_fd_ = -1;
        return false;
    }
    pipe_r_ = pfd[0];
    pipe_w_ = pfd[1];

    // Everything buffered so far goes to the real terminal first.
    std::cout.flush(); std::cerr.flush();
    std::fflush(stdout); std::fflush(stderr);

    saved_out_ = ::dup(STDOUT_FILENO);
    saved_err_ = ::dup(STDERR_FILENO);
    ::dup2(pipe_w_, STDOUT_FILENO);
    ::dup2(pipe_w_, STDERR_FILENO);
    // stdout is no longer a tty, so stdio would switch to full buffering
    // and hold lines back; keep it line-buffered like the terminal was.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    active_      = true;
    pump_thread_ = std::thread(&ConsoleTee::pump, this);
    std::cout << "[ConsoleTee] mirroring console to " << path << "\n";
    return true;
}

void ConsoleTee::pump() {
    char buf[4096];
    for (;;) {
        ssize_t n = ::read(pipe_r_, buf, sizeof buf);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (n == 0) break;  // write end closed: stop() is draining us
        write_all(saved_out_, buf, static_cast<size_t>(n));
        write_all(file_fd_,   buf, static_cast<size_t>(n));
    }
}

void ConsoleTee::stop() {
    if (!active_) return;
    std::cout.flush(); std::cerr.flush();
    std::fflush(stdout); std::fflush(stderr);

    // Point 1/2 back at the terminal, then close our last write end so the
    // pump sees EOF once the pipe is empty.
    ::dup2(saved_out_, STDOUT_FILENO);
    ::dup2(saved_err_, STDERR_FILENO);
    ::close(pipe_w_); pipe_w_ = -1;
    if (pump_thread_.joinable()) pump_thread_.join();

    ::close(pipe_r_);   pipe_r_ = -1;
    ::close(saved_out_); saved_out_ = -1;
    ::close(saved_err_); saved_err_ = -1;
    ::close(file_fd_);  file_fd_ = -1;
    active_ = false;
}

} // namespace kist
