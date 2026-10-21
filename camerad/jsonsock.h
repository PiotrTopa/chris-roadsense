// roadsense newline-delimited JSON publisher over a Unix stream socket.
//
// Header-only, single-threaded (call everything from one thread). Contract
// (docs/ARCHITECTURE.md): any number of clients connect to <rundir>/<name>.sock
// and receive every message, in order, one JSON object per line. Nothing is
// ever blocking: unsent bytes are queued per client, and a client whose queue
// exceeds 1 MiB is disconnected. Bytes received from clients are discarded.
//
//   roadsense::JsonLineServer srv;
//   srv.listen("/run/roadsense/camerad.sock");
//   srv.publish("{\"type\":\"camera\",...}");   // newline appended if missing
//   srv.service();                              // accept/flush when idle
#pragma once

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace roadsense {

class JsonLineServer {
public:
    static constexpr size_t kMaxBacklog = 1u << 20;  // 1 MiB

    JsonLineServer() = default;
    JsonLineServer(const JsonLineServer &) = delete;
    JsonLineServer &operator=(const JsonLineServer &) = delete;
    ~JsonLineServer() { close(); }

    bool listen(const std::string &path, std::string *err = nullptr) {
        close();
        sockaddr_un addr{};
        if (path.size() >= sizeof(addr.sun_path)) return fail(err, "socket path too long", ENAMETOOLONG);
        fd_ = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) return fail(err, "socket", errno);
        addr.sun_family = AF_UNIX;
        std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
        unlink(path.c_str());  // stale socket from a previous run
        if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 || ::listen(fd_, 16) != 0) {
            int e = errno;
            ::close(fd_);
            fd_ = -1;
            return fail(err, "bind/listen " + path, e);
        }
        path_ = path;
        return true;
    }

    // Closes all clients and the listening socket and unlinks the socket path.
    void close() {
        for (auto &c : clients_) ::close(c.fd);
        clients_.clear();
        if (fd_ >= 0) {
            ::close(fd_);
            unlink(path_.c_str());
        }
        fd_ = -1;
    }

    int fd() const { return fd_; }
    size_t client_count() const { return clients_.size(); }
    uint64_t disconnected_slow() const { return slow_drops_; }

    // Accept new clients, flush queued data, reap closed clients.
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
        if (clients_.empty()) return;
        const bool nl = line.empty() || line.back() != '\n';
        for (size_t i = 0; i < clients_.size();) {
            Client &c = clients_[i];
            c.out.append(line);
            if (nl) c.out.push_back('\n');
            if (!flush(c)) {
                drop(i);
                continue;
            }
            if (c.out.size() - c.off > kMaxBacklog) {
                ++slow_drops_;
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
            if (c < 0) return;  // EAGAIN or transient error
            clients_.push_back(Client{c, {}, 0});
        }
    }

    // Reads and discards client input; false if the peer closed.
    static bool drain_input(Client &c) {
        char buf[512];
        for (;;) {
            ssize_t n = recv(c.fd, buf, sizeof(buf), MSG_DONTWAIT);
            if (n > 0) continue;
            if (n == 0) return false;
            return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
        }
    }

    // Sends as much queued data as the kernel takes; false on a dead peer.
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
    std::string path_;
    std::vector<Client> clients_;
    uint64_t slow_drops_ = 0;
};

}  // namespace roadsense
