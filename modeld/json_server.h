// Newline-delimited JSON publisher with a per-client greeting.
//
// Same contract and behaviour as camerad/jsonsock.h (JsonLineServer: non-blocking, in-order,
// clients with > 1 MiB queued are dropped, client input discarded), plus one addition that
// modeld needs: a greeting line (the current `calib` message) queued to every client right
// after it is accepted (docs/ARCHITECTURE.md: calib is sent to every client when it connects).
// Kept local to modeld so camerad/ is not modified; if jsonsock.h grows an on-connect hook
// this file can go.
#pragma once

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace pocopilot {

class GreetingJsonServer {
public:
    static constexpr size_t kMaxBacklog = 1u << 20;

    GreetingJsonServer() = default;
    GreetingJsonServer(const GreetingJsonServer &) = delete;
    GreetingJsonServer &operator=(const GreetingJsonServer &) = delete;
    ~GreetingJsonServer() { close(); }

    bool listen(const std::string &path, std::string *err = nullptr) {
        close();
        sockaddr_un addr{};
        if (path.size() >= sizeof(addr.sun_path)) return fail(err, "socket path too long", ENAMETOOLONG);
        fd_ = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) return fail(err, "socket", errno);
        addr.sun_family = AF_UNIX;
        std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
        unlink(path.c_str());
        if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 || ::listen(fd_, 16) != 0) {
            int e = errno;
            ::close(fd_);
            fd_ = -1;
            return fail(err, "bind/listen " + path, e);
        }
        path_ = path;
        return true;
    }

    void close() {
        for (auto &c : clients_) ::close(c.fd);
        clients_.clear();
        if (fd_ >= 0) {
            ::close(fd_);
            unlink(path_.c_str());
        }
        fd_ = -1;
    }

    // Line queued to each newly accepted client (newline appended if missing). Empty = none.
    void set_greeting(const std::string &line) {
        greeting_ = line;
        if (!greeting_.empty() && greeting_.back() != '\n') greeting_.push_back('\n');
    }

    size_t client_count() const { return clients_.size(); }

    void service() {
        accept_all();
        for (size_t i = 0; i < clients_.size();) {
            if (!drain_input(clients_[i]) || !flush(clients_[i])) {
                drop(i);
                continue;
            }
            ++i;
        }
    }

    void publish(const std::string &line) {
        service();
        const bool nl = line.empty() || line.back() != '\n';
        for (size_t i = 0; i < clients_.size();) {
            Client &c = clients_[i];
            c.out.append(line);
            if (nl) c.out.push_back('\n');
            if (!flush(c) || c.out.size() - c.off > kMaxBacklog) {
                drop(i);
                continue;
            }
            ++i;
        }
    }

private:
    struct Client {
        int fd;
        std::string out;
        size_t off = 0;
    };
    static bool fail(std::string *err, const std::string &what, int e) {
        if (err) *err = what + ": " + strerror(e);
        return false;
    }
    void accept_all() {
        if (fd_ < 0) return;
        for (;;) {
            int c = accept4(fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (c < 0) return;
            clients_.push_back(Client{c, greeting_, 0});
        }
    }
    static bool drain_input(Client &c) {
        char buf[512];
        for (;;) {
            ssize_t n = recv(c.fd, buf, sizeof(buf), MSG_DONTWAIT);
            if (n > 0) continue;
            if (n == 0) return false;
            return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
        }
    }
    static bool flush(Client &c) {
        while (c.off < c.out.size()) {
            ssize_t n = send(c.fd, c.out.data() + c.off, c.out.size() - c.off, MSG_DONTWAIT | MSG_NOSIGNAL);
            if (n > 0) {
                c.off += size_t(n);
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
            if (n < 0 && errno == EINTR) continue;
            return false;
        }
        if (c.off == c.out.size()) {
            c.out.clear();
            c.off = 0;
        } else if (c.off > 65536 && c.off * 2 > c.out.size()) {
            c.out.erase(0, c.off);
            c.off = 0;
        }
        return true;
    }
    void drop(size_t i) {
        ::close(clients_[i].fd);
        clients_[i] = std::move(clients_.back());
        clients_.pop_back();
    }

    int fd_ = -1;
    std::string path_, greeting_;
    std::vector<Client> clients_;
};

}  // namespace pocopilot
